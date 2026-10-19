# Stage 1.1–1.3 simulator

Checks the first three Stage 1 items. Gazebo is not required.

The map is `student_ws/maps/rb3302_map.pgm`. Letter centres are simulation coordinates: the free rectangle is split into 6 columns and 4 rows, A-F north, S at the south-west cell. Measured A-X clicks are not filled in yet.

1. Manual grid input: spawn letter + heading, exactly four goal letters.
2. That map is loaded and the robot is drawn at the spawn cell.
3. A* plans from the spawn centre to the first goal. The path stays off occupied cells.

```bash
cd stage1_sim/..
python3 -m unittest stage1_sim.test_stage1
python3 -m stage1_sim --start S --bearing E --goals T,N,H,A --out stage1_preview.png
```

Green marker is goal 1 (the planned target). Orange markers are goals 2–4. The blue triangle is the robot; its nose is the heading. The red line is the A* path.

Gazebo is not required for these three checks. Localisation (AMCL) and driving are Stage 1.4 and 1.5.
