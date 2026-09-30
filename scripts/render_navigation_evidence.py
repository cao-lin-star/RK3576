#!/usr/bin/env python3
"""Render closed-bag map-frame evidence. Does not initialize ROS or publish."""
import argparse
from datetime import datetime
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.transforms import Affine2D
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import analyze_navigation_trial as audit


def last_message(folder, topic, cutoff):
    newest=None
    for file in sorted(folder.glob('*.db3')):
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro',uri=True) as db:
            row=db.execute('select id,type from topics where name=?',(topic,)).fetchone()
            if not row:continue
            msg=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? '
                           'order by timestamp desc limit 1',(row[0],int(cutoff*1e9))).fetchone()
            if msg and (newest is None or msg[0]>newest[0]):
                newest=(msg[0],deserialize_message(msg[1],get_message(row[1])))
    if newest is None:raise RuntimeError('missing '+topic)
    return newest[0]/1e9,newest[1]


def background(ax,msg,limits):
    values=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
    colors=np.where(values<0,1,np.where(values>=50,2,0))
    colormap=ListedColormap(['#fbfbf6','#d6d9dc','#3d4750'])
    origin=msg.info.origin.position
    angle=audit.yaw(msg.info.origin.orientation)
    transform=Affine2D().rotate(angle).translate(origin.x,origin.y)+ax.transData
    ax.imshow(colors,origin='lower',extent=(0,msg.info.width*msg.info.resolution,
              0,msg.info.height*msg.info.resolution),cmap=colormap,
              norm=BoundaryNorm([-.5,.5,1.5,2.5],3),interpolation='nearest',transform=transform)
    ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:]);ax.set_aspect('equal',adjustable='box')
    ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)')
    ax.grid(color='#8294a0',alpha=.2,linewidth=.5)
    ax.set_facecolor('#d6d9dc')


def heading_arrow(ax,pose,color='#151e27',length=.45):
    x,y,theta=pose['x'],pose['y'],pose['yaw']
    ax.add_patch(plt.Circle((x,y),.22,fill=False,color=color,linewidth=1.7,zorder=6))
    ax.arrow(x,y,length*np.cos(theta),length*np.sin(theta),width=.025,
             head_width=.15,head_length=.16,color=color,length_includes_head=True,zorder=7)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--before',default='2026-09-22T10:04:34+08:00')
    p.add_argument('--after',default='2026-09-22T10:04:35+08:00')
    args=p.parse_args()
    series,counts=audit.read(args.bag)
    report=audit.report(series,counts)
    goals=report['goals']
    if len(goals)<2:raise RuntimeError('requires outbound and return actions')
    times=[datetime.fromisoformat(v).timestamp() for v in (args.before,args.after)]
    _,grid=last_message(args.bag,'/map',times[-1])
    if grid.header.frame_id!='map':raise RuntimeError('static map not in map frame')
    plans=[last_message(args.bag,'/plan',t) for t in times]
    feedback=series['/navigate_to_pose/_action/feedback']
    routes=[[v for _,v in feedback if v['uuid']==g['uuid']] for g in goals[:2]]
    poses=[]
    for t in times:
        ft,fmsg=last_message(args.bag,'/navigate_to_pose/_action/feedback',t)
        if fmsg.feedback.current_pose.header.frame_id!='map':
            raise RuntimeError('action feedback pose not in map frame')
        cp=fmsg.feedback.current_pose.pose
        poses.append((ft,dict(x=cp.position.x,y=cp.position.y,yaw=audit.yaw(cp.orientation))))
    for _,plan in plans:
        if plan.header.frame_id!='map':raise RuntimeError('plan not in map frame')
    all_points=[(p['x'],p['y']) for route in routes for p in route]
    all_points += [(p.pose.position.x,p.pose.position.y) for _,plan in plans for p in plan.poses]
    xs,ys=zip(*all_points)
    limits=(min(xs)-.8,max(xs)+.8,min(ys)-.8,max(ys)+.8)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
    fig=plt.figure(figsize=(17,11),facecolor='white')
    gs=fig.add_gridspec(2,2,width_ratios=[1.05,1],left=.06,right=.97,top=.89,bottom=.11,hspace=.25,wspace=.19)
    overview=fig.add_subplot(gs[:,0]);before=fig.add_subplot(gs[0,1]);after=fig.add_subplot(gs[1,1])
    colors=['#007aab','#d85932']
    background(overview,grid,limits)
    for route,label,color in zip(routes,['Outbound actual trajectory','Return actual trajectory'],colors):
        overview.plot([p['x'] for p in route],[p['y'] for p in route],color=color,lw=1.8,alpha=.85,label=label,zorder=4)
    start=routes[0][0];target=routes[0][-1];staging=routes[1][-1]
    overview.scatter([start['x']],[start['y']],marker='s',s=70,color='#163c26',zorder=7,label='Start')
    overview.scatter([target['x']],[target['y']],marker='*',s=190,color='#77125e',zorder=7,label='Outbound goal')
    overview.scatter([staging['x']],[staging['y']],marker='D',s=55,color='#bb6800',zorder=7,label='Return staging point')
    heading_arrow(overview,start,'#163c26',.55)
    overview.set_title('Actual map-frame trajectories\nOne outbound goal and one return goal',fontweight='bold')
    overview.legend(loc='upper left',fontsize=9,framealpha=.95)
    details=[]
    for ax,(pt,plan),(ft,pose),color,caption in zip((before,after),plans,poses,colors,('Before route flip','After route flip')):
        background(ax,grid,limits)
        xy=[(p.pose.position.x,p.pose.position.y) for p in plan.poses]
        length=sum(np.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(xy,xy[1:]))
        aim=next((q for q in xy if np.hypot(q[0]-xy[0][0],q[1]-xy[0][1])>=.5),xy[-1])
        path_heading=np.arctan2(aim[1]-xy[0][1],aim[0]-xy[0][0])
        ax.plot([q[0] for q in xy],[q[1] for q in xy],lw=2.4,color=color,zorder=4,label='Global plan')
        heading_arrow(ax,pose)
        ax.scatter([xy[-1][0]],[xy[-1][1]],marker='*',s=160,color='#77125e',zorder=7)
        ax.set_title(caption+' | '+audit.wall(pt)[11:23]+' CST\n'
                     +f'Plan {length:.2f} m; initial path heading {np.degrees(path_heading):.1f} deg',fontweight='bold')
        ax.text(.02,.02,'Black circle / arrow: robot position / heading',transform=ax.transAxes,
                fontsize=8,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
        details.append(dict(plan_timestamp=audit.wall(pt),pose_timestamp=audit.wall(ft),
                            length_m=length,path_heading_deg=float(np.degrees(path_heading)),robot_pose=pose))
    fig.suptitle('Navigation evidence: repeated route reversal under the same goal',fontsize=20,fontweight='bold',y=.972)
    fig.text(.5,.935,'Closed recording, 22 Sep 2026 | The two plans on the right keep the same destination.',ha='center',fontsize=12,color='#475666')
    fig.text(.06,.055,'Trajectories and robot poses: NavigateToPose feedback already expressed in map coordinates.\n'
             f'No asynchronous odom-to-map conversion is used. Latest samples before {args.before} / {args.after}.\n'
             'White = known free; gray = unknown; dark = static occupied. Dynamic costmap / suspected-glass obstacles are not shown.',
             fontsize=9,color='#475666')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=170,facecolor='white')
    print(dict(output=str(args.output),goal_uuids=[g['uuid'] for g in goals[:2]],snapshots=details))


if __name__=='__main__':main()
