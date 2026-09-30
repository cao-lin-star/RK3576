import secrets
import pytest
from rk3576_footbath_mobile.logic import clamp_command, hash_pin, normalize_mapping_scan_source, occupancy_png, target_is_clear, verify_pin

def test_mapping_scan_source_allowlist():
    assert normalize_mapping_scan_source(None)=="fused"
    assert normalize_mapping_scan_source(" HIGH ")=="high"
    with pytest.raises(ValueError): normalize_mapping_scan_source("scan_low")

def test_velocity_limits_and_nonfinite():
    assert clamp_command(2.0,-2.0)==(0.30,-0.80)
    with pytest.raises(ValueError): clamp_command(float('nan'),0.0)

def test_pin_hash():
    salt=secrets.token_hex(16); expected=hash_pin('123456',salt)
    assert verify_pin('123456',salt,expected)
    assert not verify_pin('654321',salt,expected)

def test_goal_clearance_rejects_unknown_and_obstacle():
    data=[0]*100; assert target_is_clear(data,10,10,0.1,0,0,0.5,0.5,0.1)
    data[55]=100; assert not target_is_clear(data,10,10,0.1,0,0,0.5,0.5,0.1)
    data[55]=-1; assert not target_is_clear(data,10,10,0.1,0,0,0.5,0.5,0.1)

def test_png_signature():
    assert occupancy_png([0,100,-1,50],2,2).startswith(b'\x89PNG\r\n\x1a\n')

def test_gateway_polling_is_cached():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    source=(root/"rk3576_footbath_mobile"/"gateway.py").read_text(encoding="utf-8")
    assert 'count_publishers("/map")' not in source
    assert "self.grid is not None" in source
    assert "self.child.poll() is None" in source
    assert "now-self.last_pose_lookup>=0.20" in source
    assert "if status>=400:" in source

def test_map_view_controls_and_inverse_transform_contract():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    html=(root/"web"/"index.html").read_text(encoding="utf-8")
    js=(root/"web"/"app.js").read_text(encoding="utf-8")
    for control in ("zoomIn","zoomOut","rotateLeft","rotateRight","resetView","viewText"):
        assert f'id="{control}"' in html
    assert "function viewMetrics(c)" in js
    assert "function changeZoom(factor)" in js
    assert "function rotateMap(degrees)" in js
    assert "function resetMapView()" in js
    assert "mapView.rotation+(state?.map?.origin_yaw||0)-pose.yaw" in js
    assert "(-sx*v.sin+sy*v.cos)/v.scale" in js
    assert "setTimeout(loop,1000)" in js
    assert "lastMapLoad>4000" in js

def test_mobile_disconnect_only_releases_manual_control():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    source=(root/"rk3576_footbath_mobile"/"gateway.py").read_text(encoding="utf-8")
    watchdog=source.split("def _watchdog(self):",1)[1].split("def _stop_child(self):",1)[0]
    assert "self._release_manual()" in watchdog
    assert 'self._service("stop")' not in watchdog
    assert "self._cancel_nav()" not in watchdog
    assert "autonomous task remains under local supervision" in watchdog

def test_exploration_actions_have_real_feedback_and_explicit_estop():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    source=(root/"rk3576_footbath_mobile"/"gateway.py").read_text(encoding="utf-8")
    html=(root/"web"/"index.html").read_text(encoding="utf-8")
    js=(root/"web"/"app.js").read_text(encoding="utf-8")
    assert "_finish_service_call" in source and '"/api/emergency_stop"' in source

def test_mobile_map_management_contract():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    source=(root/"rk3576_footbath_mobile"/"gateway.py").read_text(encoding="utf-8")
    html=(root/"web"/"index.html").read_text(encoding="utf-8")
    js=(root/"web"/"app.js").read_text(encoding="utf-8")
    assert '"map_entries":self.map_store.entries' in source
    assert '"/api/maps/rename":"map_rename"' in source
    assert '"/api/maps/delete":"map_delete"' in source
    assert 'id="mapsManage"' in html and 'id="mapManagerList"' in html
    assert "function renderMapManager()" in js
    assert "移入回收站" in js

def test_saved_map_name_opens_preview_contract():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    source=(root/"rk3576_footbath_mobile"/"gateway.py").read_text(encoding="utf-8")
    js=(root/"web"/"app.js").read_text(encoding="utf-8")
    assert 'path=="/api/maps/preview"' in source
    assert "gateway.map_store.preview_data" in source
    assert 'title.onclick=()=>viewSavedMap(entry)' in js
    assert "savedMapPreview" in js

def test_mobile_mapping_scan_source_contract():
    root=__import__("pathlib").Path(__file__).resolve().parents[1]
    source=(root/"rk3576_footbath_mobile"/"gateway.py").read_text(encoding="utf-8")
    html=(root/"web"/"index.html").read_text(encoding="utf-8")
    js=(root/"web"/"app.js").read_text(encoding="utf-8")
    assert '"/scan_mapping_fused"' in source
    assert 'qos_profile_sensor_data' in source
    assert 'mapping_scan_source:={requested_source}' in source
    assert '"mapping_scan_source":self.mapping_scan_source' in source
    assert 'id="mappingScanSource"' in html
    assert 'mapping_scan_source:$("#mappingScanSource").value' in js