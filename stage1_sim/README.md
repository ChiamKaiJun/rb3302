# Stage 1.1–1.3 simulator

Checks the first three Stage 1 items without Gazebo or ROS:

1. Manual grid input: spawn letter + heading, exactly four goal letters.
2. Map loaded, robot drawn at that spawn, goals drawn on their squares.
3. A* plans from the spawn centre to the first goal. The path stays off the walls.

```bash
cd stage1_sim/..
python3 -m unittest stage1_sim.test_stage1
python3 -m stage1_sim --start S --bearing E --goals T,N,H,A --out stage1_preview.png
```

Green marker is goal 1 (the planned target). Orange markers are goals 2–4. The blue triangle is the robot; its nose is the heading. The red line is the A* path.

Gazebo is not required for these three checks. Localisation (AMCL) and driving are Stage 1.4 and 1.5.
