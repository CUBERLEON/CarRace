#pragma once
#pragma comment(linker, "/STACK:1000000000")

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

#define PI 3.14159265358979323846
#define DOUBLE_PI 6.28318530717958647692
#define _USE_MATH_DEFINES
#define DEBUG
#define OPPOSITE_DIRECTION_COST 8

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <set>
#include <algorithm>
#include <cstring>
#include <utility>

#define pii pair<int, int>
#define pdd pair<double, double>
#define PX pos.first
#define PY pos.second
#define sqr(a) (a)*(a)

#define RIGHT 0
#define UP 1
#define LEFT 2
#define DOWN 3

const double EPS = 1e-7;
const int MAX = 20;
const int dx[] { 1, 0, -1, 0 };
const int dy[] { 0, -1, 0, 1 };

using namespace model;
using namespace std;

template <typename T, typename U> pair<T, U> operator+ (const pair<T, U>& l, const pair<T, U>& r) { return{ l.first + r.first, l.second + r.second }; }
template <typename T, typename U> pair<T, U> operator+= (pair<T, U>& l, const pair<T, U>& r) { l.first += r.first; l.second += r.second; return l; }
template <typename T, typename U> pair<T, U> operator- (const pair<T, U>& l, const pair<T, U>& r) { return{ l.first - r.first, l.second - r.second }; }
template <typename T, typename U> pair<T, U> operator-= (pair<T, U>& l, const pair<T, U>& r) { l.first -= r.first; l.second -= r.second; return l; }
template <typename T, typename U> pair<T, U> operator* (pair<T, U>& l, const double r) { return{ l.first * r, l.second * r }; }
template <typename T, typename U> pair<T, U> operator*= (pair<T, U>& l, const double r) { l.first *= r; l.second *= r; return l; }
template <typename T, typename U> pair<T, U> operator* (const double l, const pair<T, U>& r) { return{ r.first * l, r.second * l }; }
template <typename T, typename U> pair<T, U> operator/ (pair<T, U>& l, const double r) { return{ l.first / r, l.second / r }; }
template <typename T, typename U> pair<T, U> operator/= (pair<T, U>& l, const double r) { l.first /= r; l.second /= r; return l; }
template <typename T, typename U> pair<T, U> operator/ (const double l, const pair<T, U>& r) { return{ r.first / l, r.second / l }; }

struct Tile {
	pii parent;
	int direction;
	pii pos;
	Tile(pii parent = { 0, 0 }, int direction = 0, pii pos = { 0, 0 }) : parent(parent), direction(direction), pos(pos) {}
	bool operator== (const Tile& another) { return parent == another.parent && direction == another.direction && pos == another.pos; }
	bool operator!= (const Tile& another) {	return !(*this == another); }
};

struct Controller {
	double wheelTurn;
	double enginePower;
	bool isBreaking;
	bool useNitro;
	Controller(double wheelTurn = 0., double enginePower = 0., bool isBreaking = false, bool useNitro = false) :
		wheelTurn(wheelTurn), enginePower(enginePower), isBreaking(isBreaking), useNitro(useNitro) {}
};

struct CarState {
	int carType;

	Controller controller;

	pdd pos;
	pdd speed;
	double angularSpeed;
	double angle;

	double baseAngSpeed;

	CarState(int carType = 0, Controller controller = Controller(), pdd pos = { 0., 0. }, pdd speed = { 0., 0. },
			 double angularSpeed = 0., double angle = 0., double baseAngSpeed = 0.) :
		carType(carType), controller(controller), pos(pos), speed(speed), 
		angularSpeed(angularSpeed), angle(angle), baseAngSpeed(baseAngSpeed) {}

	void print() {
		printf("pos: (%lf, %lf) speed: (%lf, %lf) wheel: (%lf) engine: (%lf) angSpeed: (%lf) angle: (%lf) breaking: %d\n",
				PX, PY, speed.first, speed.second, controller.wheelTurn, controller.enginePower, angularSpeed, angle, controller.isBreaking);
	}
};

enum Mode {
	NORMAL,
	STUCK
};

class MyStrategy : public Strategy {
public:
    MyStrategy();
    void move(const model::Car& self, const model::World& world, const model::Game& game, model::Move& move);
private:
	//strategy objects
	const Car* m_self;
	const World* m_world;
	const Game* m_game;
	Move* m_move;

	//system variables
	bool m_isInitialized;
	bool m_isMode2x2;

	//my car variables
	double m_mySpeed;
	int m_myMovingDirection;
	pdd m_myMovingVector;
	pii m_myTile;
	pii m_myLastTile;
	vector<pdd> m_myCarCorners;
	pdd m_myLastPos;
	pdd m_myPos;

	//active mode variables
	int m_activeMode;
	int m_modeTicks;

	//world variables
	vector<vector<TileType> > m_tiles;
	double m_tileMargin;
	double m_tileSize;
	int m_waypointsCnt;
	pii m_nextWaypointPos;

	//route variables
	vector<Tile> m_route;
	vector<pdd> m_routeRealXY;
	vector<int> m_routeDistToCorner;
	int m_curRouteIndex;
	int m_nextRouteIndex;

	//physics
	const int m_physIters = 10;
	const double m_physDt = 1. / m_physIters;
	double m_carAccelFactor[2][2]; // [i][j] i(CarType) j(0 - forward, 1 - rear)
	double m_frictMul;
	double m_longFrict;
	double m_crossFrict;
	double m_carRotFactor;
	double m_rotFrictMul;
	double m_angFrict;
	double m_wheelCPT;
	double m_engineCPT;

	Controller m_myCarController;
	CarState m_myCarState;

	vector<CarState> m_myStates;
	int m_myStateIndex;
	bool m_isNeededStatesUpdate;

	CarState m_otherCarState[3];

	void init();
	void updateValues();

	void simulatePhysics(CarState &state, const Controller &newController);

	void computeStates(const CarState& base, const int curRouteIndex);
	pair<double, vector<CarState>> computeStatesAux(const CarState& base, const int curRouteIndex, const int step, const int stepsCnt, const int* k, const double* wheelStep);
	Controller getUpdatedController(const Controller& old, const Controller& dest);

	inline int getIndex(int x, int y);
	inline int getIndex(const pii& p);

	inline pii getTile(double x1, double y1);
	inline pii getTile(const pdd& p);
	inline TileType getTileType(const pii& p);

	inline pii getCarTile(const Car& car);
	inline bool isCarInTile(const Car& car, const pii& tile, double k = 0.);
	vector<pdd> getCarCorners(const Car& car);
	inline int getCarMovingDirection(const Car& car);
	inline pdd getCarSpeed(const Car& car);
	inline double getCarSpeedModule(const Car& car);
	inline pdd getCarPos(const Car& car);
	inline CarState toCarState(const Car& car);

	inline pdd getLongUnitVector(double angle);
	inline pdd getCrossUnitVector(double angle);
	vector<pdd> getCorners(const pdd& pos, double width, double height, double angle);
	bool isPointOnRoad(const pdd& p);
	inline bool isPointInTile(const pdd& p, const pii& tile, double k = 0.);

	inline double cross(const pdd& a, const pdd& b);
	inline double dot(const pdd& a, const pdd& b);
	inline int sgn(double a);
	inline double length(const pdd& a);
	inline double limit(double val, double lim) { return max(-lim, min(lim, val)); }
	inline void normalizeAngle(double& angle);
	inline double getAngleBetween(const pdd& a, const pdd& b);
	inline double getAngle(const pdd& a);
	int isPointInPolygon(const vector<pdd>& polygon, const pdd& p);
	int isPointInRectangle(const pdd& a, const pdd& b, const pdd& p);
	inline pdd rotate(const pdd& a, double angle);
	inline double distance(const pdd& a, const pdd& b);

	inline int getDirection(double angle);
	inline int getDirectionChangeCost(int a, int b);
	inline bool isDirectionAccessible(TileType myTile, int directon);
	vector<int> getAccessibleDirections(TileType tile);

	vector<Tile> optimizeRoute(vector<Tile> &old);
	vector<Tile> computeRoute(const pii& start, int direction, const pii& dest, int parentDirection = -1);
	void computeRouteValues(const vector<Tile> &route, bool cyclic, vector<pdd>& realXY, vector<int>& isCorner);
	pdd computeNextXY();
	void updateRoute(int parentDirection = -1, int direction = -1);

	void setMode(int mode, bool forced = false);

	void normalMode();
	void stuckMode();

	bool isStuck();

	void nitroController();
	void fireController();
	void breakingController();

	void debugPoint(const pdd& p, int r = 0, int g = 0, int b = 0);
	void debugLine(const pdd& st, const pdd& en, int r = 0, int g = 0, int b = 0);
};

#endif
