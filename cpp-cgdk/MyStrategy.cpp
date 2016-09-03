#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <bits/stdc++.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <set>
#include <algorithm>

#define pii pair<int,int>
#define vi vector<int>
#define PX pos.first
#define PY pos.second

using namespace model;
using namespace std;

struct Tile {
	pii parent;
	int direction;
	pii pos;
	Tile(pii parent = { 0, 0 }, int direction = 0, pii pos = { 0, 0 }) : parent(parent), direction(direction), pos(pos) {}
};

const double EPS = 1e-9;
int needInit = 1;
double prevX=-1000, prevY=-1000;
int numoftics=0,numofticks=0;
double curPow = 1.0, curAngle = 0;
const int MAX = 20;
const Car* m_self;
const World* m_world;
const Game* m_game;

vector<Tile> route;
int nextWaypoint;

#define RIGHT 0
#define UP 1
#define LEFT 2
#define DOWN 3

pair<int, int> getTile(double x1, double y1) {
	for (int x = 0; x < m_world->getWidth(); ++x) {
		for (int y = 0; y <= m_world->getHeight(); ++y) {
			if (x1 + EPS >= x * m_game->getTrackTileSize() && x1 <= (x + 1) * m_game->getTrackTileSize() + EPS &&
				y1 + EPS >= y * m_game->getTrackTileSize() && y1 <= (y + 1) * m_game->getTrackTileSize() + EPS &&
				m_world->getTilesXY()[x][y] != TileType::EMPTY)
				return{ x, y };
		}
	}
}

pair<int, int> getCarTile(const Car& car) {
	return getTile(car.getX(), car.getY());
}

inline int getDirection(double angle) {
	if (angle < 0) angle += 2 * PI;
	if (angle >= PI / 4. && angle < 3.*PI / 4.) return 3;
	if (angle >= 3.*PI / 4. && angle < 5.*PI / 4.) return 2;
	if (angle >= 5.*PI / 4. && angle < 7.*PI / 4.) return 1;
	return 0;
}

inline int getDirectionChangeCost(int Old, int New) {
	if (Old == New) return 1;
	if (abs(Old - New) == 1 || abs(Old - New) == 3) return 2;
	return 8;
}

inline int getIndex(int x, int y) { return x * MAX + y; }
inline int getIndex(const pii& p) { return getIndex(p.first, p.second); }

int dx[] = { 1, 0, -1, 0 };
int dy[] = { 0, -1, 0, 1 };


int isDirectionAccessible(TileType myTile, int directon) {
	int a[4] = { 0 };
	switch (myTile)	{
	case EMPTY:	{
					break;
	}
	case VERTICAL: {
					   a[UP] = a[DOWN] = 1;
					   break;
	};
	case HORIZONTAL: {
						 a[LEFT] = a[RIGHT] = 1;
						 break;
	};
	case LEFT_TOP_CORNER: {
							  a[RIGHT] = a[DOWN] = 1;
							  break;
	}
	case RIGHT_TOP_CORNER: {
							   a[LEFT] = a[DOWN] = 1;
							   break;
	}
	case LEFT_BOTTOM_CORNER: {
								 a[RIGHT] = a[UP] = 1;
								 break;
	}
	case RIGHT_BOTTOM_CORNER: {
								  a[LEFT] = a[UP] = 1;
								  break;
	}
	case LEFT_HEADED_T: {
							a[LEFT] = a[UP] = a[DOWN] = 1;
							break;
	}
	case RIGHT_HEADED_T: {
							 a[RIGHT] = a[UP] = a[DOWN] = 1;
							 break;
	}
	case TOP_HEADED_T: {
						   a[UP] = a[LEFT] = a[RIGHT] = 1;
						   break;
	}
	case BOTTOM_HEADED_T: {
							  a[DOWN] = a[LEFT] = a[RIGHT] = 1;
							  break;
	}
	case CROSSROADS: 
	{
						 a[LEFT] = a[RIGHT] = a[UP] = a[DOWN] = 1;
						 break;
	}
	case UNKNOWN:
	{
					a[LEFT] = a[RIGHT] = a[UP] = a[DOWN] = 1;
					break;
	}
	default: {
				 printf("Error in initializing directions array\n");
				 break;
	}
	}
	return a[directon];
}

vector<Tile> computePath(const pii &st, int direction, const pii &dest) {
	set< pair<int, int> > s;
	Tile tiles[MAX*MAX];
	int d[MAX*MAX];
	int used[MAX*MAX];

	memset(d, 1, sizeof(d));
	memset(used, 0, sizeof(used));

	Tile start({ -1, -1 }, direction, st);
	d[getIndex(start.pos)] = 0;
	s.insert({ 0, getIndex(start.pos) });
	tiles[getIndex(start.pos)] = start;

	while (!s.empty()) {
		int curD = s.begin()->first;
		Tile cur = tiles[s.begin()->second];
		s.erase(s.begin());
		used[getIndex(cur.pos)] = 1;
		if (cur.pos == dest)
			break;

		for (int i = 0; i < 4; i++) {
			if (!isDirectionAccessible(m_world->getTilesXY()[cur.PX][cur.PY], i))
			{
				continue;
			}
			pii nextPos = { dx[i] + cur.PX, dy[i] + cur.PY };
			if (nextPos.first < 0 || nextPos.second < 0 ||
				nextPos.first >= m_world->getWidth() || nextPos.second >= m_world->getHeight() ||
				used[getIndex(nextPos)])
				continue;

			int newD = curD + getDirectionChangeCost(cur.direction, i);
			if (newD < d[getIndex(nextPos)]) {
				auto it = s.find({ d[getIndex(nextPos)], getIndex(nextPos) });
				if (it != s.end())
					s.erase(it);
				s.insert({ newD, getIndex(nextPos) });

				d[getIndex(nextPos)] = newD;
				tiles[getIndex(nextPos)] = Tile(cur.pos, i, nextPos);
			}
		}
	}

	vector<Tile> ans;
	Tile cur = tiles[getIndex(dest)];
	ans.push_back(cur);
	while (cur.parent != make_pair(-1, -1)) {
		cur = tiles[getIndex(cur.parent)];
		ans.push_back(cur);
	}
	reverse(ans.begin(), ans.end());

	return ans;
}

void init() {
	needInit = 0;
	route.clear();
	int lastDirection = getDirection(m_self->getAngle());
	for (int i = 0; i < m_world->getWaypoints().size(); i++) {
		int j = (i + 1) % m_world->getWaypoints().size();
		pii start(m_world->getWaypoints()[i][0], m_world->getWaypoints()[i][1]),
			finish(m_world->getWaypoints()[j][0], m_world->getWaypoints()[j][1]);

		vector<Tile> path = computePath(start, lastDirection, finish);
		lastDirection = path[path.size() - 1].direction;
		for (int k = 0; k < path.size() - 1 + (j == 0); k++)
			route.push_back(path[k]);
	}
	nextWaypoint = 1;
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
	m_self = &self;
	m_world = &world;
	m_game = &game;
	if (needInit)
		init();
	if (world.getTick() < game.getInitialFreezeDurationTicks()) return;
	if (route[nextWaypoint].pos == getCarTile(self))
		nextWaypoint = (nextWaypoint + 1) % route.size();
	int doublenextWaypoint = (nextWaypoint + 1) % route.size();

	double nextWaypointX = (route[nextWaypoint].pos.first + 0.5) * game.getTrackTileSize();
	double nextWaypointY = (route[nextWaypoint].pos.second + 0.5) * game.getTrackTileSize();

	double cornerTileOffset = 0.35 * game.getTrackTileSize();

	nextWaypointX -= cornerTileOffset * dx[route[nextWaypoint].direction];
	nextWaypointY -= cornerTileOffset * dy[route[nextWaypoint].direction];
	nextWaypointX += cornerTileOffset * dx[route[doublenextWaypoint].direction];
	nextWaypointY += cornerTileOffset * dy[route[doublenextWaypoint].direction];
	double angleToWaypoint = self.getAngleTo(nextWaypointX, nextWaypointY);
	double speedModule = hypot(self.getSpeedX(), self.getSpeedY());

	double nowX = self.getX(), nowY = self.getY();
	if (numoftics > 0) numoftics--; else
	if (numofticks) 
	{
		numofticks--;
		curPow = 1.0;
		curAngle = angleToWaypoint * 32.0 / PI;
	}
	else
	if (abs(nowX - prevX)<0.21 && abs(nowY - prevY)<0.21 && world.getTick()>200) curPow = -1.0, numoftics = 100, numofticks=100, curAngle =-( (angleToWaypoint) * 32.0 / PI), cout << "prev";
	else
	{
		prevX = nowX, prevY = nowY;
		if (curPow == -1.0) curPow = 1.0;
		curAngle = angleToWaypoint * 32.0 / PI;
	}

	move.setWheelTurn(curAngle);
	move.setEnginePower(curPow);
	
	if (speedModule * speedModule * abs(angleToWaypoint) > 2.5 * 2.5 * PI * 6.5) {
		move.setBrake(true);
	}
	else {
				bool ifShmalyaty = false;
		 		if (world.getTick()>220) move.setUseNitro(true);
				vector<Car> q = world.getCars();
				for (int i = 0; i<q.size(); i++)
				{
					double enemyX = q[i].getX();
					double enemyY = q[i].getY();
					double toEnemy = self.getAngleTo(enemyX, enemyY);
					if ((toEnemy - curAngle) <= 0.005 && (toEnemy - curAngle) >= 0.0) cout << "Way:"<<toEnemy<<' '<<curAngle<<endl, ifShmalyaty = true;
				}
				if (world.getTick()>250) move.setSpillOil(true);
				move.setThrowProjectile(ifShmalyaty);
	}
}

MyStrategy::MyStrategy() { }
