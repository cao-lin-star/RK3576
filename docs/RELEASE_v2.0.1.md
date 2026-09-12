# v2.0.1：修复 Linux 启动脚本换行符

日期：2026-09-12。基于双雷达 v2.0.0，不修改建图、导航或底盘控制逻辑。

板端 footbath_mobile_start 曾包含 CRLF，Bash 将 pipefail 后的回车识别为选项内容，手机服务 status=2 并反复重启。修正 LF 后用户已确认手动建图、自动建图、导航正常。

根因：旧属性只为 .sh 等后缀明确规定 LF，无扩展名启动脚本没有 eol 规则；Windows core.autocrlf=true 下工作副本变成 CRLF。Git 索引中的 LF 不代表工作副本或导出包必然为 LF。

修复：文本文件默认 text=auto eol=lf，覆盖无扩展名脚本；保留 .ps1/.bat/.cmd 的 CRLF 和图像二进制规则。修复本地脚本换行，并检查发布归档中所有 Shell 脚本为 LF。

v2.0.0 标签保持不变。重新打包应使用 v2.0.1。此次提交不重新部署板端；现场已执行的启动脚本修复继续有效。
