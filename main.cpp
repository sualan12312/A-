#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;

int width, height;
vector<vector<bool>> isFree;

bool loadMap(const string& filename) {
    ifstream file(filename);
    if (!file) return false;

    string line;
    while (getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line == "map") break;
        if (line.find("height") == 0) height = stoi(line.substr(7));
        if (line.find("width") == 0) width = stoi(line.substr(6));
    }

    isFree.assign(height, vector<bool>(width));
    for (int y = 0; y < height; y++) {
        getline(file, line);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        for (int x = 0; x < width; x++) {
            isFree[y][x] = (line[x] == '.');
        }
    }
    return true;
}

vector<pair<int, int>> astar(int sx, int sy, int gx, int gy,
                             vector<pair<int, int>>& visited) {
    vector<vector<double>> g(height, vector<double>(width, 1e9));
    vector<vector<pair<int, int>>> parent(height, vector<pair<int, int>>(width, {-1, -1}));
    vector<vector<bool>> closed(height, vector<bool>(width, false));

    priority_queue<pair<double, pair<int, int>>,
                   vector<pair<double, pair<int, int>>>,
                   greater<>> pq;

    g[sy][sx] = 0;
    pq.push({0, {sx, sy}});

    int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};
    double cost[] = {1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414};

    while (!pq.empty()) {
        auto pos = pq.top().second; pq.pop();
        int x = pos.first, y = pos.second;

        if (closed[y][x]) continue;
        closed[y][x] = true;
        visited.push_back({x, y});

        if (x == gx && y == gy) break;

        for (int i = 0; i < 8; i++) {
            int nx = x + dx[i], ny = y + dy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            if (!isFree[ny][nx] || closed[ny][nx]) continue;
            if (i >= 4 && (!isFree[y][nx] && !isFree[ny][x])) continue;

            double new_g = g[y][x] + cost[i];
            if (new_g < g[ny][nx]) {
                g[ny][nx] = new_g;
                parent[ny][nx] = {x, y};
                double h = max(abs(nx - gx), abs(ny - gy)) +
                           0.414 * min(abs(nx - gx), abs(ny - gy));
                pq.push({new_g + h, {nx, ny}});
            }
        }
    }

    vector<pair<int, int>> path;
    if (g[gy][gx] >= 1e9) return path;

    int x = gx, y = gy;
    while (x != -1) {
        path.push_back({x, y});
        auto p = parent[y][x];
        x = p.first;
        y = p.second;
    }
    reverse(path.begin(), path.end());
    return path;
}

void drawResult(int sx, int sy, int gx, int gy,
                const vector<pair<int, int>>& visited,
                const vector<pair<int, int>>& path,
                const string& output) {
    int scale = 3;
    cv::Mat img(height * scale, width * scale, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            if (!isFree[y][x])
                cv::rectangle(img, {x * scale, y * scale},
                              {(x + 1) * scale - 1, (y + 1) * scale - 1},
                              {0, 0, 0}, -1);

    for (auto [x, y] : visited)
        cv::rectangle(img, {x * scale, y * scale},
                      {(x + 1) * scale - 1, (y + 1) * scale - 1},
                      {200, 200, 200}, -1);

    for (auto [x, y] : path)
        cv::rectangle(img, {x * scale, y * scale},
                      {(x + 1) * scale - 1, (y + 1) * scale - 1},
                      {0, 0, 255}, -1);

    cv::circle(img, {sx * scale + scale / 2, sy * scale + scale / 2}, scale * 2, {0, 255, 0}, -1);
    cv::circle(img, {gx * scale + scale / 2, gy * scale + scale / 2}, scale * 2, {255, 0, 0}, -1);

    cv::imwrite(output, img);
    cout << "Saved: " << output << endl;
}

int main(int argc, char** argv) {
    string mapfile = "../map.map";
    int sx = 10, sy = 10, gx = 240, gy = 240;
    string output = "result.png";

    if (argc >= 6) {
        mapfile = argv[1];
        sx = stoi(argv[2]); sy = stoi(argv[3]);
        gx = stoi(argv[4]); gy = stoi(argv[5]);
        if (argc >= 7) output = argv[6];
    }

    if (!loadMap(mapfile)) {
        cout << "Failed to load map: " << mapfile << endl;
        return 1;
    }
    cout << "Map: " << width << "x" << height << endl;
    cout << "Start: (" << sx << "," << sy << ")  Goal: (" << gx << "," << gy << ")" << endl;

    if (!isFree[sy][sx] || !isFree[gy][gx]) {
        cout << "Start or goal is blocked!" << endl;
        return 1;
    }

    vector<pair<int, int>> visited;
    auto path = astar(sx, sy, gx, gy, visited);

    if (path.empty()) {
        cout << "No path found! Visited " << visited.size() << " nodes." << endl;
    } else {
        double total = 0;
        for (size_t i = 1; i < path.size(); i++) {
            int dx = abs(path[i].first - path[i-1].first);
            int dy = abs(path[i].second - path[i-1].second);
            total += (dx + dy == 2) ? 1.414 : 1.0;
        }
        cout << "Path found! Cost: " << total << ", Length: " << path.size()
             << " nodes, visited " << visited.size() << " nodes." << endl;
    }

    drawResult(sx, sy, gx, gy, visited, path, output);

    return 0;
}
