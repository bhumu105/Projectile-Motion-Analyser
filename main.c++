#define GL_SILENCE_DEPRECATION
#include <GLUT/glut.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

struct Point {
    float t;
    float x;
    float y;
};

struct SimulationState {
    float speed = 30.0f;
    float angleDeg = 45.0f;
    float initialHeight = 0.0f;
    float dt = 0.02f;
    float gravity = 9.81f;
    float flightTime = 0.0f;
    float range = 0.0f;
    float maxHeight = 0.0f;
    std::vector<Point> points;
};

SimulationState gSim;
int gWinW = 1280;
int gWinH = 720;
float gTime = 0.0f;
bool gPaused = false;
bool gShowGuide = true;
bool gShowTrajectory = false;

struct ButtonRect {
    float x;
    float y;
    float w;
    float h;
};

std::string formatStat(const std::string& label, float value, const std::string& unit, int precision = 2);

float readPositive(const std::string& prompt) {
    float value = 0.0f;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value > 0.0f) {
            return value;
        }
        std::cout << "Please enter a positive number.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

float readNonNegative(const std::string& prompt) {
    float value = 0.0f;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value >= 0.0f) {
            return value;
        }
        std::cout << "Please enter a non-negative number.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

void buildSimulation(SimulationState& sim) {
    sim.points.clear();

    const float pi = 3.14159265358979323846f;
    const float angleRad = sim.angleDeg * pi / 180.0f;
    const float vx = sim.speed * std::cos(angleRad);
    const float vy = sim.speed * std::sin(angleRad);

    sim.maxHeight = sim.initialHeight;
    float prevT = 0.0f;
    float prevX = 0.0f;
    float prevY = sim.initialHeight;
    bool hasPrev = false;

    for (float t = 0.0f;; t += sim.dt) {
        const float x = vx * t;
        const float y = sim.initialHeight + vy * t - 0.5f * sim.gravity * t * t;

        if (y < 0.0f && hasPrev) {
            // Interpolate the exact impact point so the rendered arc reaches the ground.
            const float denom = (prevY - y);
            const float alpha = (denom != 0.0f) ? (prevY / denom) : 0.0f;
            const float hitT = prevT + alpha * (t - prevT);
            const float hitX = prevX + alpha * (x - prevX);
            sim.points.push_back({hitT, hitX, 0.0f});
            sim.flightTime = hitT;
            sim.range = hitX;
            break;
        }

        sim.points.push_back({t, x, y});
        if (y > sim.maxHeight) {
            sim.maxHeight = y;
        }
        prevT = t;
        prevX = x;
        prevY = y;
        hasPrev = true;
    }
}

void drawText(float x, float y, void* font, const std::string& text, float r, float g, float b) {
    glColor3f(r, g, b);
    glRasterPos2f(x, y);
    for (char c : text) {
        glutBitmapCharacter(font, c);
    }
}

void drawBackground() {
    glBegin(GL_QUADS);
    glColor3f(0.06f, 0.10f, 0.19f);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(static_cast<float>(gWinW), 0.0f);
    glColor3f(0.02f, 0.02f, 0.05f);
    glVertex2f(static_cast<float>(gWinW), static_cast<float>(gWinH));
    glVertex2f(0.0f, static_cast<float>(gWinH));
    glEnd();

    glBegin(GL_QUADS);
    glColor4f(0.98f, 0.58f, 0.20f, 0.32f);
    glVertex2f(0.0f, static_cast<float>(gWinH) * 0.82f);
    glVertex2f(static_cast<float>(gWinW), static_cast<float>(gWinH) * 0.82f);
    glColor4f(0.15f, 0.15f, 0.20f, 0.0f);
    glVertex2f(static_cast<float>(gWinW), static_cast<float>(gWinH));
    glVertex2f(0.0f, static_cast<float>(gWinH));
    glEnd();
}

void drawGround(float margin, float plotBottom, float plotWidth) {
    const float y = plotBottom;
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glColor3f(0.30f, 0.88f, 0.50f);
    glVertex2f(margin, y);
    glVertex2f(margin + plotWidth, y);
    glEnd();
}

void drawGrid(float margin, float plotBottom, float plotWidth, float plotHeight) {
    glLineWidth(1.0f);
    glColor4f(0.58f, 0.67f, 0.82f, 0.15f);
    glBegin(GL_LINES);
    const int verticalLines = 10;
    const int horizontalLines = 8;
    for (int i = 0; i <= verticalLines; ++i) {
        const float x = margin + plotWidth * static_cast<float>(i) / static_cast<float>(verticalLines);
        glVertex2f(x, plotBottom);
        glVertex2f(x, plotBottom + plotHeight);
    }
    for (int i = 0; i <= horizontalLines; ++i) {
        const float y = plotBottom + plotHeight * static_cast<float>(i) / static_cast<float>(horizontalLines);
        glVertex2f(margin, y);
        glVertex2f(margin + plotWidth, y);
    }
    glEnd();
}

void drawLaunchAngleArc(float margin, float plotBottom, float plotWidth, float plotHeight) {
    if (gSim.points.empty()) {
        return;
    }
    (void)plotWidth;

    const float maxY = std::max(1.0f, gSim.maxHeight * 1.08f);
    const float originX = margin;
    const float originY = plotBottom + (gSim.initialHeight / maxY) * plotHeight;
    const float arcRadius = 48.0f;
    const float displayAngleDeg = std::clamp(gSim.angleDeg, 0.0f, 180.0f);
    const float angleRad = displayAngleDeg * 3.1415926535f / 180.0f;

    glLineWidth(1.5f);
    glBegin(GL_LINES);
    glColor4f(0.9f, 0.92f, 1.0f, 0.65f);
    glVertex2f(originX, originY);
    glVertex2f(originX + 75.0f, originY);
    glVertex2f(originX, originY);
    glVertex2f(originX + 75.0f * std::cos(angleRad), originY + 75.0f * std::sin(angleRad));
    glEnd();

    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    glColor3f(1.0f, 0.72f, 0.23f);
    const int arcSegments = 40;
    for (int i = 0; i <= arcSegments; ++i) {
        const float a = angleRad * static_cast<float>(i) / static_cast<float>(arcSegments);
        glVertex2f(originX + arcRadius * std::cos(a), originY + arcRadius * std::sin(a));
    }
    glEnd();

    drawText(originX + arcRadius + 12.0f, originY + 8.0f, GLUT_BITMAP_HELVETICA_12,
             formatStat("theta", displayAngleDeg, "deg", 1), 1.0f, 0.86f, 0.45f);
}

Point samplePointAtTime(float timeSec) {
    if (gSim.points.empty()) {
        return {0.0f, 0.0f, 0.0f};
    }
    if (timeSec <= 0.0f) {
        return gSim.points.front();
    }
    if (timeSec >= gSim.flightTime) {
        return gSim.points.back();
    }
    for (std::size_t i = 1; i < gSim.points.size(); ++i) {
        const Point& a = gSim.points[i - 1];
        const Point& b = gSim.points[i];
        if (b.t >= timeSec) {
            const float span = b.t - a.t;
            const float alpha = (span > 0.0f) ? ((timeSec - a.t) / span) : 0.0f;
            return {
                timeSec,
                a.x + alpha * (b.x - a.x),
                a.y + alpha * (b.y - a.y),
            };
        }
    }
    return gSim.points.back();
}

std::size_t lastVisibleIndex(float clampedTime) {
    if (gSim.points.empty()) {
        return 0;
    }
    std::size_t idx = 0;
    while (idx + 1 < gSim.points.size() && gSim.points[idx + 1].t <= clampedTime) {
        ++idx;
    }
    return idx;
}

void drawTrajectory(float margin, float plotBottom, float plotWidth, float plotHeight, Point& currentOut) {
    const float maxX = std::max(1.0f, gSim.range);
    const float maxY = std::max(1.0f, gSim.maxHeight * 1.08f);
    const float clampedTime = std::clamp(gTime, 0.0f, gSim.flightTime);

    if (gSim.points.empty()) {
        currentOut = {0.0f, 0.0f, 0.0f};
        return;
    }

    currentOut = samplePointAtTime(clampedTime);
    const std::size_t trailEnd = lastVisibleIndex(clampedTime);

    // Full predicted path
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    glColor4f(0.52f, 0.72f, 1.0f, gShowTrajectory ? 0.35f : 0.18f);
    for (const auto& p : gSim.points) {
        if (p.y < 0.0f) {
            continue;
        }
        const float xNorm = p.x / maxX;
        const float yNorm = p.y / maxY;
        const float sx = margin + xNorm * plotWidth;
        const float sy = plotBottom + yNorm * plotHeight;
        glVertex2f(sx, sy);
    }
    glEnd();

    if (!gShowTrajectory) {
        return;
    }

    // Actual traveled arc path (up to current time)
    glLineWidth(3.2f);
    glBegin(GL_LINE_STRIP);
    glColor3f(0.39f, 0.92f, 1.0f);
    for (std::size_t i = 0; i <= trailEnd; ++i) {
        const auto& p = gSim.points[i];
        const float sx = margin + (p.x / maxX) * plotWidth;
        const float sy = plotBottom + (std::max(0.0f, p.y) / maxY) * plotHeight;
        glVertex2f(sx, sy);
    }
    const float cTrailX = margin + (currentOut.x / maxX) * plotWidth;
    const float cTrailY = plotBottom + (std::max(0.0f, currentOut.y) / maxY) * plotHeight;
    glVertex2f(cTrailX, cTrailY);
    glEnd();

    // Mark apex and landing
    const float apexTime = (gSim.speed * std::sin(gSim.angleDeg * 3.1415926535f / 180.0f)) / gSim.gravity;
    const Point apex = samplePointAtTime(std::clamp(apexTime, 0.0f, gSim.flightTime));
    const float ax = margin + (apex.x / maxX) * plotWidth;
    const float ay = plotBottom + (std::max(0.0f, apex.y) / maxY) * plotHeight;
    const Point impact = gSim.points.back();
    const float ix = margin + (impact.x / maxX) * plotWidth;
    const float iy = plotBottom;

    glPointSize(7.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.58f, 0.60f);
    glVertex2f(ax, ay);
    glColor3f(0.65f, 1.0f, 0.70f);
    glVertex2f(ix, iy);
    glEnd();

    drawText(ax + 8.0f, ay + 10.0f, GLUT_BITMAP_HELVETICA_12, "apex", 1.0f, 0.70f, 0.72f);
    drawText(ix - 54.0f, iy + 12.0f, GLUT_BITMAP_HELVETICA_12, "impact", 0.70f, 1.0f, 0.74f);

    const float bx = margin + (currentOut.x / maxX) * plotWidth;
    const float by = plotBottom + (std::max(0.0f, currentOut.y) / maxY) * plotHeight;
    const float pulse = 8.0f + 2.4f * std::sin(gTime * 10.0f);

    glPointSize(12.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.84f, 0.20f);
    glVertex2f(bx, by);
    glEnd();

    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glColor4f(1.0f, 0.84f, 0.20f, 0.5f);
    for (int i = 0; i < 50; ++i) {
        const float a = 2.0f * 3.1415926535f * static_cast<float>(i) / 50.0f;
        glVertex2f(bx + pulse * std::cos(a), by + pulse * std::sin(a));
    }
    glEnd();
}

std::string formatStat(const std::string& label, float value, const std::string& unit, int precision) {
    std::ostringstream out;
    out << label << ": " << std::fixed << std::setprecision(precision) << value << " " << unit;
    return out.str();
}

ButtonRect buildToggleButton(float margin, float plotBottom, float plotHeight) {
    const float top = plotBottom + plotHeight;
    return {static_cast<float>(gWinW) - margin - 192.0f, top + 10.0f, 192.0f, 42.0f};
}

bool pointInButton(const ButtonRect& btn, float px, float py) {
    return px >= btn.x && px <= (btn.x + btn.w) && py >= btn.y && py <= (btn.y + btn.h);
}

void drawButton(const ButtonRect& btn, const std::string& label, bool active) {
    glBegin(GL_QUADS);
    if (active) {
        glColor4f(0.24f, 0.76f, 0.99f, 0.92f);
    } else {
        glColor4f(0.16f, 0.20f, 0.30f, 0.88f);
    }
    glVertex2f(btn.x, btn.y);
    glVertex2f(btn.x + btn.w, btn.y);
    glVertex2f(btn.x + btn.w, btn.y + btn.h);
    glVertex2f(btn.x, btn.y + btn.h);
    glEnd();

    glLineWidth(1.6f);
    glBegin(GL_LINE_LOOP);
    glColor4f(0.82f, 0.90f, 1.0f, 0.95f);
    glVertex2f(btn.x, btn.y);
    glVertex2f(btn.x + btn.w, btn.y);
    glVertex2f(btn.x + btn.w, btn.y + btn.h);
    glVertex2f(btn.x, btn.y + btn.h);
    glEnd();

    drawText(btn.x + 20.0f, btn.y + 16.0f, GLUT_BITMAP_HELVETICA_12, label, 0.95f, 0.98f, 1.0f);
}

void drawHud(float margin, float plotBottom, float plotHeight) {
    const float top = plotBottom + plotHeight;
    const ButtonRect toggleButton = buildToggleButton(margin, plotBottom, plotHeight);
    const float currentT = std::clamp(gTime, 0.0f, gSim.flightTime);
    const Point current = samplePointAtTime(currentT);
    const float angleRad = gSim.angleDeg * 3.1415926535f / 180.0f;
    const float vx = gSim.speed * std::cos(angleRad);
    const float vy = gSim.speed * std::sin(angleRad) - gSim.gravity * currentT;
    const float vMag = std::sqrt(vx * vx + vy * vy);

    drawText(margin, top + 58.0f, GLUT_BITMAP_HELVETICA_18, "PROJECTILE MOTION VISUALIZER", 0.93f, 0.95f, 1.0f);
    drawText(margin, top + 36.0f, GLUT_BITMAP_HELVETICA_12, formatStat("Speed", gSim.speed, "m/s"), 0.82f, 0.87f, 1.0f);
    drawText(margin + 210.0f, top + 36.0f, GLUT_BITMAP_HELVETICA_12, formatStat("Angle", gSim.angleDeg, "deg"), 0.82f, 0.87f, 1.0f);
    drawText(margin + 420.0f, top + 36.0f, GLUT_BITMAP_HELVETICA_12, formatStat("Start Height", gSim.initialHeight, "m"), 0.82f, 0.87f, 1.0f);

    drawText(margin, top + 14.0f, GLUT_BITMAP_HELVETICA_12, formatStat("Flight Time", gSim.flightTime, "s"), 0.70f, 0.96f, 0.78f);
    drawText(margin + 210.0f, top + 14.0f, GLUT_BITMAP_HELVETICA_12, formatStat("Range", gSim.range, "m"), 0.70f, 0.96f, 0.78f);
    drawText(margin + 420.0f, top + 14.0f, GLUT_BITMAP_HELVETICA_12, formatStat("Max Height", gSim.maxHeight, "m"), 0.70f, 0.96f, 0.78f);
    drawText(margin + 620.0f, top + 36.0f, GLUT_BITMAP_HELVETICA_12, formatStat("t", currentT, "s"), 0.92f, 0.91f, 0.70f);
    drawText(margin + 620.0f, top + 14.0f, GLUT_BITMAP_HELVETICA_12, formatStat("x", current.x, "m"), 0.92f, 0.91f, 0.70f);
    drawText(margin + 770.0f, top + 14.0f, GLUT_BITMAP_HELVETICA_12, formatStat("y", current.y, "m"), 0.92f, 0.91f, 0.70f);
    drawText(margin + 770.0f, top + 36.0f, GLUT_BITMAP_HELVETICA_12, formatStat("v", vMag, "m/s"), 0.92f, 0.91f, 0.70f);
    drawText(margin + 920.0f, top + 36.0f, GLUT_BITMAP_HELVETICA_12, formatStat("vx", vx, "m/s"), 0.92f, 0.91f, 0.70f);
    drawText(margin + 920.0f, top + 14.0f, GLUT_BITMAP_HELVETICA_12, formatStat("vy", vy, "m/s"), 0.92f, 0.91f, 0.70f);
    drawButton(toggleButton, gShowTrajectory ? "Hide Trajectory" : "Show Trajectory", gShowTrajectory);

    if (gShowGuide) {
        drawText(margin, 26.0f, GLUT_BITMAP_HELVETICA_12,
                 "Predicted arc=dim, actual traveled arc=bright  |  Click button: show/hide  |  Space: pause/resume  |  R: restart",
                 0.80f, 0.82f, 0.88f);
    } else {
        drawText(margin, 26.0f, GLUT_BITMAP_HELVETICA_12, "H: show help", 0.80f, 0.82f, 0.88f);
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    drawBackground();

    const float margin = 86.0f;
    const float plotBottom = 82.0f;
    const float plotWidth = static_cast<float>(gWinW) - margin * 2.0f;
    const float plotHeight = static_cast<float>(gWinH) - 220.0f;

    Point current{};
    drawGrid(margin, plotBottom, plotWidth, plotHeight);
    drawGround(margin, plotBottom, plotWidth);
    drawLaunchAngleArc(margin, plotBottom, plotWidth, plotHeight);
    drawTrajectory(margin, plotBottom, plotWidth, plotHeight, current);
    drawHud(margin, plotBottom, plotHeight);

    glutSwapBuffers();
}

void reshape(int width, int height) {
    gWinW = std::max(640, width);
    gWinH = std::max(480, height);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, static_cast<double>(gWinW), 0.0, static_cast<double>(gWinH));
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void keyboard(unsigned char key, int, int) {
    if (key == 27) {
        std::exit(0);
    }
    if (key == ' ') {
        gPaused = !gPaused;
    }
    if (key == 'r' || key == 'R') {
        gTime = 0.0f;
    }
    if (key == 's' || key == 'S') {
        gShowTrajectory = !gShowTrajectory;
        if (gShowTrajectory) {
            gTime = 0.0f;
            gPaused = false;
        }
    }
    if (key == 'h' || key == 'H') {
        gShowGuide = !gShowGuide;
    }
}

void mouse(int button, int state, int x, int y) {
    if (button != GLUT_LEFT_BUTTON || state != GLUT_DOWN) {
        return;
    }

    const float margin = 86.0f;
    const float plotBottom = 82.0f;
    const float plotHeight = static_cast<float>(gWinH) - 220.0f;
    const ButtonRect toggleButton = buildToggleButton(margin, plotBottom, plotHeight);

    const float mouseX = static_cast<float>(x);
    const float mouseY = static_cast<float>(gWinH - y);
    if (pointInButton(toggleButton, mouseX, mouseY)) {
        gShowTrajectory = !gShowTrajectory;
        if (gShowTrajectory) {
            gTime = 0.0f;
            gPaused = false;
        }
        glutPostRedisplay();
    }
}

void update(int) {
    if (!gPaused && gShowTrajectory) {
        gTime += 0.016f;
        if (gTime > gSim.flightTime + 0.6f) {
            gTime = 0.0f;
        }
    }
    glutPostRedisplay();
    glutTimerFunc(16, update, 0);
}

int main(int argc, char** argv) {
    std::cout << "Projectile Motion Visualizer\n";
    std::cout << "----------------------------\n";
    gSim.speed = readPositive("Initial speed (m/s): ");
    gSim.angleDeg = readNonNegative("Launch angle (degrees): ");
    gSim.initialHeight = readNonNegative("Initial height (m): ");
    gSim.dt = readPositive("Simulation step (s, e.g. 0.02): ");

    buildSimulation(gSim);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(gWinW, gWinH);
    glutCreateWindow("Projectile Motion Visualizer");

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutTimerFunc(16, update, 0);

    glutMainLoop();
    return 0;
}
