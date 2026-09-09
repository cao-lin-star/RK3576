# RK3576 version baselines

Repository verification: git diff --cached --check passed; 90 existing WSL pytest cases passed (health, dock geometry/state, mobile logic/maps/saved home/return transition, safety logic). Full suite collection requires the generated explore_lite_msgs package unavailable in this test environment; no new ARM64 build was performed.

Repository: https://github.com/cao-lin-star/RK3576.git

2026-09-09: v1.0.0 preserves b52f38da8466fa4a4e1813c4232e33079c65d698 (high-lidar mapping; low lidar may still serve avoidance). v2.0.0 imports the current dual-lidar worktree from parent snapshot 02b366e. Both generations were accepted by the user as stable historical baselines.

Branches: codex/single-lidar from v1.0.0; main and codex/dual-lidar at v2.0.0; codex/camera-docking starts from v2.0.0 without new camera implementation. Preserve tags and release future camera work under a new version.

Board evidence: gateway.py, dock_motion.py, dual_lidar_fusion_node.cpp and mapping.launch.py matched the worktree by SHA256 on 2026-09-09. Installed Python/launch files also matched. The board has no Git metadata and went offline before a full comparison. No complete binary audit or fresh hardware acceptance test is claimed. No board deployment or movement was performed.

Use this independent repository for future development. Historical parent-repository/WSL copies do not synchronize automatically. Build on RK ARM64. Maps and credentials remain outside Git. The board needs a future backed-up Git migration before git pull can be used. F407 remains separately managed in F407_controll.
