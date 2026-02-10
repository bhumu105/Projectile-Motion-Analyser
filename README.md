# Projectile Motion Visualizer (C++)

Interactive projectile simulation built with C++ and OpenGL/GLUT.

The app opens in its own window, animates projectile flight, and visualizes both:
- the full predicted trajectory (dim arc)
- the actual traveled path in real time (bright arc)

It also includes launch-angle (`theta`) labeling, apex/impact markers, and live telemetry.

## Features

- Real-time projectile animation
- Full arc rendering to exact ground impact
- Launch-angle arc with `theta` label
- Live stats:
  - `t`, `x`, `y`
  - `v`, `vx`, `vy`
  - flight time, range, max height
- Clickable `Show Trajectory` / `Hide Trajectory` button
- Clean visual UI with gradient background and HUD

## Physics Model

Uses standard no-drag projectile equations:

- `x(t) = v0 * cos(theta) * t`
- `y(t) = y0 + v0 * sin(theta) * t - 0.5 * g * t^2`

With:
- `v0` = initial speed
- `theta` = launch angle
- `y0` = initial height
- `g = 9.81 m/s^2`

## Requirements

- macOS
- Apple Clang (`g++`)
- OpenGL + GLUT frameworks (available via Command Line Tools)

## Build

```bash
g++ -std=c++17 -Wall -Wextra -pedantic main.c++ -framework OpenGL -framework GLUT -o projectile_window
```

## Run

```bash
./projectile_window
```

Then enter:
- initial speed (m/s)
- launch angle (degrees)
- initial height (m)
- simulation step (s)

## Controls

- Mouse click on button: show/hide trajectory
- `S`: show/hide trajectory
- `Space`: pause/resume
- `R`: restart animation
- `H`: toggle help text
- `Esc`: quit

## Project Structure

- `main.c++`: full simulator, renderer, UI controls

## Notes

- This version uses a no-air-resistance model.
- Ground-impact point is interpolated so the arc ends exactly at `y = 0`.

