#include "MyStrategy.h"

MyStrategy::MyStrategy() {
	m_isInitialized = 0;
	m_isMode2x2 = 0;
}

int MyStrategy::getIndex(int x, int y) { return x * MAX + y; }

int MyStrategy::getIndex(const pii& p) { return getIndex(p.first, p.second); }

pii MyStrategy::getTile(double x1, double y1) {
	int x = x1 / m_tileSize;
	int y = y1 / m_tileSize;
	if (x < 0 || x >= m_world->getWidth() ||
		y < 0 || y >= m_world->getHeight())
		return { -1, -1 };
	return {x, y};
}

pii MyStrategy::getTile(const pdd& p) {
	return getTile(p.first, p.second);
}

TileType MyStrategy::getTileType(const pii& p) {
	if (p == make_pair(-1, -1))
		return EMPTY;
	return m_tiles[p.first][p.second];
}

pii MyStrategy::getCarTile(const Car& car) {
	return getTile(car.getX(), car.getY());
}

bool MyStrategy::isCarInTile(const Car& car, const pii& tile, double k) {
	return isPointInTile({ car.getX(), car.getY() }, tile, k);
}

bool MyStrategy::isPointInTile(const pdd& p, const pii& tile, double k) {
	return p.first + EPS >= (tile.first - k) * m_game->getTrackTileSize() && p.first <= (tile.first + 1. + k) * m_game->getTrackTileSize() + EPS &&
		   p.second + EPS >= (tile.second - k) * m_game->getTrackTileSize() && p.second <= (tile.second + 1. + k) * m_game->getTrackTileSize() + EPS;
}

bool MyStrategy::isPointOnRoad(const pdd& p) {
	pii tile = getTile(p);

	if (getTileType(tile) == EMPTY)
		return false;
	pdd v = { p.first - tile.first * m_tileSize,
			  p.second - tile.second * m_tileSize };
	if (v.first < 0 || v.second < 0 ||
		v.first > m_tileSize || v.second > m_tileSize)
		return false;

	vector<int> directions = getAccessibleDirections(getTileType(tile));
	int ax[] = { 1, 1, 0, 0 };
	int ay[] = { 1, 0, 0, 1 };

	for (int i = 0; i < 4; ++i) {
		int j = (i + 1) % 4;

		pdd vi = { ax[i] * m_tileSize, ay[i] * m_tileSize },
			vj = { ax[j] * m_tileSize, ay[j] * m_tileSize };

		if (!directions[i]) {
			pdd vi1 = { vi.first - dx[i] * m_tileMargin, vi.second - dy[i] * m_tileMargin };
			if (isPointInRectangle(vi1, vj, v))
				return false;
		}
		if (directions[i] && directions[j]) {
			if (distance(vj, v) <= m_tileMargin + EPS)
				return false;
		}
		if (!directions[i] && !directions[j]) {
			pdd cv = { vj.first - 2. * dx[i] * m_tileMargin - 2. * dx[j] * m_tileMargin, vj.second - 2. * dy[i] * m_tileMargin - 2. * dy[j] * m_tileMargin };
			if (isPointInRectangle(vj, cv, v))
			if (distance(v, cv) + EPS > m_tileMargin)
				return false;
		}
	}
	return true;
}

vector<pdd> MyStrategy::getCarCorners(const Car& car) {
	return getCorners({ car.getX(), car.getY() }, car.getWidth(), car.getHeight(), car.getAngle());
}

int MyStrategy::getCarMovingDirection(const Car& car) {
	if (getCarSpeedModule(car) < EPS)
		return 0;
	double angle = car.getAngleTo(car.getX() + car.getSpeedX(), car.getY() + car.getSpeedY());
	if (abs(angle) > PI / 2.)
		return -1;
	return 1;
}

pdd MyStrategy::getCarSpeed(const Car& car) {
	return { car.getSpeedX(), car.getSpeedY()};
}

double MyStrategy::getCarSpeedModule(const Car& car) {
	return length(getCarSpeed(car));
}

pdd MyStrategy::getCarPos(const Car& car) {
	return {car.getX(), car.getY()};
}

CarState MyStrategy::toCarState(const Car& car) {
	return CarState(car.getType(), Controller(car.getWheelTurn(), car.getEnginePower(), false), getCarPos(car), getCarSpeed(car),
					car.getAngularSpeed(), car.getAngle(), m_carRotFactor * car.getWheelTurn() * dot(getCarSpeed(car), getLongUnitVector(car.getAngle())));
}

pdd MyStrategy::getLongUnitVector(double angle) {
	return rotate({ 1., 0. }, angle);
}

pdd MyStrategy::getCrossUnitVector(double angle) {
	return rotate({ 0., 1. }, angle);
}

vector<pdd> MyStrategy::getCorners(const pdd& pos, double width, double height, double angle) {
	vector<pdd> corners(4);
	pdd v1 = rotate({ width / 2., height / 2. }, angle);
	pdd v2 = rotate({ -height / 2., width / 2. }, -angle);
	corners[0] = { PX + v1.first, PY + v1.second };
	corners[1] = { PX + v2.second, PY + v2.first };
	corners[2] = { PX - v1.first, PY - v1.second };
	corners[3] = { PX - v2.second, PY - v2.first };
	return corners;
}

double MyStrategy::cross(const pdd& a, const pdd& b) {
	return a.first * b.second - a.second * b.first;
}

double MyStrategy::dot(const pdd& a, const pdd& b) {
	return a.first * b.first + a.second * b.second;
}

inline int MyStrategy::sgn(double a) {
	return (a > 0.) - (a < 0.);
}

double MyStrategy::length(const pdd& a) {
	return sqrt(a.first * a.first + a.second * a.second);
}

void MyStrategy::normalizeAngle(double& angle) {
	while (angle > PI) {
		angle -= DOUBLE_PI;
	}
	while (angle < -PI) {
		angle += DOUBLE_PI;
	}
}

double MyStrategy::getAngleBetween(const pdd& a, const pdd& b) {
	double dot = a.first*b.first + a.second*b.second;
	double det = a.first*b.second - a.second*b.first;
	double angle = atan2(det, dot);
	if (angle < 0.)
		angle += 2. * PI;
	return min(angle, 2. * PI - angle);
}

double MyStrategy::getAngle(const pdd& a) {
	double angle = atan2(a.second, a.first);
	if (angle < 0.)
		angle += 2. * PI;
	return angle;
}

int MyStrategy::isPointInPolygon(const vector<pdd>& polygon, const pdd& p) {
	if (polygon.size() < 3)
		return 0;

	int a[3] = { 0 };
	for (int i = 0; i < polygon.size(); ++i) {
		int j = (i + 1) % polygon.size();
		++a[1 + sgn(cross({ p.first - polygon[i].first, p.second - polygon[i].second },
						  { polygon[j].first - polygon[i].first, polygon[j].second - polygon[i].second }))];
	}
	if (a[0] && a[2])
		return 0;
	if (a[1])
		return 2;
	return 1;
}

int MyStrategy::isPointInRectangle(const pdd& a, const pdd& b, const pdd& p) {
	int sx = sgn(a.first - p.first) * sgn(b.first - p.first);
	int sy = sgn(a.second- p.second) * sgn(b.second - p.second);
	if (sx > 0 || sy > 0)
		return 0;
	if (sx == 0 || sy == 0)
		return 2;
	return 1;
}

pdd MyStrategy::rotate(const pdd& a, double angle) {
	double cosa = cos(angle), sina = sin(angle);
	return { cosa * a.first - sina * a.second,
			 sina * a.first + cosa * a.second };
}

int MyStrategy::getDirection(double angle) {
	if (angle < 0) angle += 2 * PI;
	if (angle > PI / 4. && angle <= 3.*PI / 4.) return 3;
	if (angle > 3.*PI / 4. && angle <= 5.*PI / 4.) return 2;
	if (angle > 5.*PI / 4. && angle <= 7.*PI / 4.) return 1;
	return 0;
}

int MyStrategy::getDirectionChangeCost(int a, int b) {
	if (a == b) return 2;
	if (abs(a - b) == 1 || abs(a - b) == 3) return 3;
	return OPPOSITE_DIRECTION_COST;
}

bool MyStrategy::isDirectionAccessible(TileType myTile, int directon) {
	return getAccessibleDirections(myTile)[directon];
}

vector<int> MyStrategy::getAccessibleDirections(TileType tile) {
	vector<int> a(4, 0);
	switch (tile)	{
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
	case CROSSROADS: case UNKNOWN: {
						 a[LEFT] = a[RIGHT] = a[UP] = a[DOWN] = 1;
						 break;
	}
	default: {
				 printf("Error in initializing directions array\n");
				 break;
	}
	}
	return a;
}

vector<Tile> MyStrategy::optimizeRoute(vector<Tile> &old) {
	vector<Tile> ans;
	ans.push_back(old[0]);
	for (int i = 1; i < old.size(); i++) {
		if ((i + 2 < old.size() && old[i + 2].direction != old[i].direction) ||
			(i + 1 < old.size() && old[i + 1].direction != old[i].direction) ||
			old[i - 1].direction != old[i].direction && ans[0].pos != old[i].pos)
			ans.push_back(old[i]);
	}
	if (ans.back() != old.back() && old.back().pos != ans.front().pos)
		ans.push_back(old.back());
	return ans;
}

vector<Tile> MyStrategy::computeRoute(const pii& st, int direction, const pii& dest, int parentDirection) {
	set< pair<int, int> > s;

	Tile tiles[MAX*MAX];
	int d[MAX*MAX];
	int used[MAX*MAX];

	memset(d, 1, sizeof(d));
	memset(used, 0, sizeof(used));

	Tile start({ -1, -1 }, direction, st);
	tiles[getIndex(start.pos)] = start;
	d[getIndex(start.pos)] = 0;
	s.insert({ 0, getIndex(start.pos) });

	while (!s.empty()) {
		int curD = s.begin()->first;
		Tile cur = tiles[s.begin()->second];
		s.erase(s.begin());
		used[getIndex(cur.pos)] = 1;
		if (cur.pos == dest)
			break;

		for (int i = 0; i < 4; i++) {
			if (!isDirectionAccessible(m_tiles[cur.PX][cur.PY], i))
				continue;
			pii nextPos = { dx[i] + cur.PX, dy[i] + cur.PY };
			if (nextPos.first < 0 || nextPos.second < 0 ||
				nextPos.first >= m_world->getWidth() || nextPos.second >= m_world->getHeight() ||
				used[getIndex(nextPos)])
				continue;

			int previousDirection = (cur == start) ? parentDirection : tiles[getIndex(cur.parent)].direction;
			int newD = curD;
			if (i == previousDirection && abs(cur.direction - i) == 1)
				newD += 1;
			else
				newD += getDirectionChangeCost(cur.direction, i);
			if (isDirectionAccessible(getTileType(cur.pos), cur.direction) && abs(cur.direction - i) == 1)
				newD += 1;
			if (abs(previousDirection - i) == 2)
				newD += 5;
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
	while (cur.parent != start.parent) {
		cur = tiles[getIndex(cur.parent)];
		ans.push_back(cur);
	}
	reverse(ans.begin(), ans.end());

	return ans;
}

void MyStrategy::computeRouteValues(const vector<Tile> &route, bool cyclic, vector<pdd>& realXY, vector<int>& distanceToCorner) {
	realXY = vector<pdd>(route.size());
	distanceToCorner = vector<int>(route.size(), 1);

	double cornerOffset = 0.34 * m_game->getTrackTileSize();//0.34
	double straightOffset = 0.21 * m_game->getTrackTileSize();
	double oppositeDirectionCornerOffset = 0. * m_game->getTrackTileSize();//-0.2

	for (int i = 0; i < route.size(); ++i) {
		int j = (i + 1) % route.size();

		realXY[i] = { (m_route[i].PX + 0.5) * m_game->getTrackTileSize(), (m_route[i].PY + 0.5) * m_game->getTrackTileSize() };
		if (!cyclic && !j) continue;
// 		realXY[i].first += 0.5 * m_game->getTrackTileSize() * dx[m_route[j].direction];
// 		realXY[i].second += 0.5 * m_game->getTrackTileSize() * dy[m_route[j].direction];

		if (route[i].direction != route[j].direction) {
			distanceToCorner[i] = 0;
			realXY[i].first += cornerOffset * dx[m_route[j].direction] - cornerOffset * dx[m_route[i].direction];
			realXY[i].second += cornerOffset * dy[m_route[j].direction] - cornerOffset * dy[m_route[i].direction];
		}
	}
	for (int i = 0; i < route.size() - 2*(!cyclic); ++i) {
		int j = (i + 1) % route.size();
		int k = (i + 2) % route.size();

		if (m_route[i].direction != m_route[j].direction && getDirectionChangeCost(m_route[k].direction, m_route[i].direction) == OPPOSITE_DIRECTION_COST) {
			realXY[i].first += oppositeDirectionCornerOffset * dx[m_route[k].direction];
			realXY[i].second += oppositeDirectionCornerOffset * dy[m_route[k].direction];
		}
	}

	int tmp[MAX*MAX], ptr = 0;
	for (int i = route.size() - 2 + cyclic; i >= !cyclic; --i) {
		if (!distanceToCorner[i]) {
			for (int j = i - 1; j >= -int(route.size()); --j) {
				int k = (j + route.size()) % route.size();
				if (!cyclic && j <= 0) break;
				if (!distanceToCorner[k]) {
// 					for (int p = 0; p < ptr; ++p)
// 						distanceToCorner[tmp[p]] = min(distanceToCorner[tmp[p]], min(abs(tmp[p] - k), abs(int(route.size()) - k + tmp[p])));
					break;
				}
				tmp[ptr++] = k;
				distanceToCorner[k] = min(abs(i - k), abs(int(route.size()) - i + k));
				realXY[k].first -= straightOffset * dx[m_route[(i + 1) % route.size()].direction];
				realXY[k].second -= straightOffset * dy[m_route[(i + 1) % route.size()].direction];
			}
			ptr = 0;
		}
	}
}

pdd MyStrategy::computeNextXY() {

// 	for (int i = 1; i < m)
	return pdd();
}

void MyStrategy::updateRoute(int parentDirection, int direction)  {
	m_route.clear();
	int nextWaypoint = m_self->getNextWaypointIndex();
	int lastDirection = (direction == -1) ? getDirection(m_self->getAngle()) : direction;
	for (int i = nextWaypoint; i < nextWaypoint + 10; i++) {
		int j1 = i % m_waypointsCnt;
		int i1 = (i - 1 + m_waypointsCnt) % m_waypointsCnt;
		pii start(m_world->getWaypoints()[i1][0], m_world->getWaypoints()[i1][1]),
			finish(m_world->getWaypoints()[j1][0], m_world->getWaypoints()[j1][1]);
		if (i == nextWaypoint)
			start = m_myTile;

		vector<Tile> path = computeRoute(start, lastDirection, finish, parentDirection);
		lastDirection = path[path.size() - 1].direction;
		parentDirection = (path.size() >= 2) ? path[path.size() - 2].direction : -1;
		for (int k = i != nextWaypoint; k < path.size(); k++)
			m_route.push_back(path[k]);
	}

// 	m_route = optimizeRoute(m_route);
	computeRouteValues(m_route, false, m_routeRealXY, m_routeDistToCorner);
// 	for (int i = 2; i < m_route.size(); ++i) {
// 		if (!m_routeDistToCorner[i]) {
// 			m_nextRouteIndex = i - 1;
// 			break;
// 		}
// 	}
	m_nextRouteIndex = 1;
	m_curRouteIndex = 0;
}

void MyStrategy::setMode(int mode, bool forced) {
	if (m_activeMode == mode || (m_modeTicks < 50 && !forced))
		return;
	m_activeMode = mode;
	m_modeTicks = 0;
}

void MyStrategy::normalMode() {
// 	pdd waypointPos = m_routeRealXY[m_nextRouteIndex];
// 	pdd waypointPos = (m_routeRealXY[m_nextRouteIndex] + m_routeRealXY[m_nextRouteIndex + 1] + m_routeRealXY[m_nextRouteIndex+2]) / 3.;

// 	double bonusD = 1e9;
// 	Bonus* bonus = NULL;
// 	for (int i = 0; i < m_world->getBonuses().size(); ++i) {
// 		Bonus cur = m_world->getBonuses()[i];
// 		double curD = distance({ cur.getX(), cur.getY() }, m_myPos);
// 		if (m_routeDistToCorner[m_nextRouteIndex] >= 1 &&
// 			curD < m_tileSize && curD < bonusD &&
// 			abs(m_self->getAngleTo(cur)) < PI / 10. &&
// 			(isPointInTile({cur.getX(), cur.getY()}, m_route[m_nextRouteIndex].pos) ||
// 			isPointInTile({ cur.getX(), cur.getY() }, m_route[m_curRouteIndex].pos))) {
// 			bonus = &cur;
// 			bonusD = curD;
// 		}
// 	}
// 	if (!(bonus == NULL))
// 		dest = { bonus->getX(), bonus->getY() };

	double angleToWaypoint = m_self->getAngleTo(m_routeRealXY[m_nextRouteIndex].first, m_routeRealXY[m_nextRouteIndex].second);
// 	m_move->setWheelTurn((m_routeDistToCorner[m_nextTileIndex] ? min(1., m_routeDistToCorner[m_nextTileIndex] / 2.) : 1.) * angleToWaypoint * 32.0 / PI);
// 	double wheelTurnOnWaypoint = m_myMovingDirection * angleToWaypoint * 32. / PI;
// 	m_move->setWheelTurn(m_myMovingDirection * wheelTurnOnWaypoint * 32. / PI);
// 	m_move->setEnginePower(1.0);

	double diff = 1e-2;
	if (!m_isNeededStatesUpdate && 
		(m_myStateIndex >= 0.5 * m_myStates.size() ||
		length(m_myCarState.pos - m_myStates[m_myStateIndex].pos) > diff ||
		abs(m_myCarState.angle - m_myStates[m_myStateIndex].angle) > diff ||
		length(m_myCarState.speed - m_myStates[m_myStateIndex].speed) > diff))
		m_isNeededStatesUpdate = true;

	if (m_isNeededStatesUpdate)
		computeStates(m_myCarState, m_curRouteIndex);

	if (m_myStates.size() > 1) {
		m_move->setWheelTurn(m_myStates[m_myStateIndex + 1].controller.wheelTurn);
		m_move->setEnginePower(1.0);
// 		m_move->setBrake(m_myStates[m_myStateIndex + 1].controller.isBreaking);
// 		m_move->setUseNitro(m_myStates[m_myStateIndex + 1].controller.useNitro);
	}
	else {
		printf("Shit!");
		double wheelTurnOnWaypoint = m_myMovingDirection * angleToWaypoint * 32. / PI;
		m_move->setWheelTurn(wheelTurnOnWaypoint);
		m_move->setEnginePower(1.0);
	}

// 	m_move->setBrake(false);
// 	if (m_myMovingDirection == -1) {
// 		if (m_mySpeed > 5.5 && m_self->getEnginePower() < 0.)
// 			m_move->setBrake(true);
// 	} else {
// 		if (m_mySpeed * m_mySpeed * abs(angleToWaypoint) > 2.5 * 2.5 * PI * 7.0)
// 			m_move->setBrake(true);
// 	}
}

bool MyStrategy::isStuck() {
	if (m_mySpeed > 1.)
		return false;

	double len = 0.5 * m_tileMargin;
	int k = 5;

	double tmp = distance(m_myCarCorners[0], m_myCarCorners[3]);
	pdd v = { (m_myCarCorners[0].first - m_myCarCorners[3].first) / tmp, (m_myCarCorners[0].second - m_myCarCorners[3].second) / tmp };

	for (int i = 0; i < k; ++i) {
		pdd v1 = { (m_myCarCorners[0].first - m_myCarCorners[1].first) / (k - 1.),
				   (m_myCarCorners[0].second - m_myCarCorners[1].second) / (k - 1.) };
		pdd p = { m_myCarCorners[1].first + double(i) * v1.first + len * v.first, m_myCarCorners[1].second + double(i) * v1.second + len * v.second };
		
		debugPoint(p);

		if (!isPointOnRoad(p))
			return true;
		for (int j = 0; j < m_world->getCars().size(); ++j)
			if (isPointInPolygon(getCarCorners(m_world->getCars()[j]), p))
				return true;

	}
	return false;
}

void MyStrategy::init() {
	m_tileMargin = m_game->getTrackTileMargin();
	m_tileSize = m_game->getTrackTileSize();
	m_tiles = m_world->getTilesXY();

	m_carAccelFactor[model::BUGGY][0] = m_game->getBuggyEngineForwardPower() / m_game->getBuggyMass() * m_physDt;
	m_carAccelFactor[model::JEEP][0] = m_game->getJeepEngineForwardPower() / m_game->getJeepMass() * m_physDt;
	m_carAccelFactor[model::BUGGY][1] = -m_game->getBuggyEngineRearPower() / m_game->getBuggyMass() * m_physDt;
	m_carAccelFactor[model::JEEP][1] = -m_game->getJeepEngineRearPower() / m_game->getJeepMass() * m_physDt;
	m_frictMul = pow(1. - m_game->getCarMovementAirFrictionFactor(), m_physDt);
	m_longFrict = m_game->getCarLengthwiseMovementFrictionFactor() * m_physDt;
	m_crossFrict = m_game->getCarCrosswiseMovementFrictionFactor() * m_physDt;
	m_carRotFactor = m_game->getCarAngularSpeedFactor();
	m_rotFrictMul = pow(1. - m_game->getCarRotationAirFrictionFactor(), m_physDt);
	m_angFrict = m_game->getCarRotationFrictionFactor() * m_physDt;
	m_wheelCPT = m_game->getCarWheelTurnChangePerTick();
	m_engineCPT = m_game->getCarEnginePowerChangePerTick();

	m_myCarController = Controller();
	m_isNeededStatesUpdate = true;
	m_myStateIndex = 0;

	for (int i = 0; i < m_world->getWidth(); ++i) {
		for (int j = 0; j < m_world->getHeight(); ++j)
			if (m_world->getTilesXY()[i][j] == TileType::UNKNOWN)
				m_isMode2x2 = 1;
	}

	updateValues();
	m_myLastPos = m_myPos;
	m_myLastTile = m_myTile;

	updateRoute();

	setMode(Mode::NORMAL, true);

	m_isInitialized = 1;
}

void MyStrategy::stuckMode() {
	pdd nextWaypoint = m_routeRealXY[m_nextRouteIndex];
	double angle = m_self->getAngleTo(nextWaypoint.first, nextWaypoint.second);
	if (m_modeTicks > 150 || (abs(angle) < PI / 6. && !isStuck())) {
		setMode(Mode::NORMAL, true);
		return;
	}
	m_move->setWheelTurn(-sgn(angle));
	m_move->setEnginePower(-1.);
}

void MyStrategy::nitroController() {
	if (m_self->getRemainingNitroTicks() > 0 ||
		m_self->getRemainingOiledTicks() > 0)
		return;
	m_move->setUseNitro(false);
	bool flag = true;
	for (int i = 0; i <= 4 && m_nextRouteIndex + i < m_route.size(); i++) {
		if (abs(m_self->getAngleTo(m_routeRealXY[m_nextRouteIndex + i].first, m_routeRealXY[m_nextRouteIndex + i].second)) > PI / 8.)
			flag = false;
	}
	int freeze = 50;
	if (flag &&	m_world->getTick() > m_game->getInitialFreezeDurationTicks() + freeze)
		m_move->setUseNitro(true);
}

void MyStrategy::breakingController() {

}

double MyStrategy::distance(const pdd& a, const pdd& b) {
	return sqrt((a.first - b.first)*(a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}

void MyStrategy::fireController() {
	vector<Car> cars = m_world->getCars();

	int projectileAims[3] = { 0 };
	bool isProjectileLowHPAim = false;
	int oilAimsCnt = 0;
	for (int i = 0; i < cars.size(); ++i) {
		if (cars[i].getId() == m_self->getId() ||
			cars[i].isTeammate() ||
			cars[i].isFinishedTrack() ||
			cars[i].getDurability() < EPS) continue;

		Car &enemy = cars[i];
		int curEnemyProjectileAimsCnt = 0;

		for (int i = -1; i <= 1; ++i) {
			double projectileAngle = (m_self->getAngle() + i * m_game->getSideWasherAngle());
			double enemySteeringAngle = (enemy.getAngle() + enemy.getWheelTurn() * PI / 15.);
			
			double x1 = m_self->getX(), y1 = m_self->getY(),
				   x2 = enemy.getX(), y2 = enemy.getY();
			double a1 = cos(projectileAngle), b1 = sin(projectileAngle),
				   a2 = cos(enemySteeringAngle), b2 = sin(enemySteeringAngle);

			double d = -b1*a2 + a1 * b2,
				   dx = -(x1*b1-y1*a1)*a2 + a1*(x2*b2-y2*a2),
				   dy = (x2*b2 - y2*a2)*b1 - b2*(x1*b1 - y1*a1);

			if (abs(d) < EPS)
				continue;

			pdd aim = { dx / d, dy / d };
			if ((aim.first - m_self->getX()) * a1 < 0 ||
				(aim.second - m_self->getY()) * b1 < 0 ||
				!isPointOnRoad(aim))
				continue;

			double projectileTime = distance({ m_self->getX(), m_self->getY() }, aim) / m_game->getWasherInitialSpeed();
			pdd nextEnemyPos = { enemy.getX() + projectileTime * enemy.getSpeedX(),
								 enemy.getY() + projectileTime * enemy.getSpeedY() };

			if (abs(enemy.getAngularSpeed() * projectileTime) > PI / 4. ||
				abs(enemy.getWheelTurn()) * min(10., hypot(enemy.getSpeedX(), enemy.getSpeedY())) > 9.)
				continue;

			if (distance(aim, nextEnemyPos) < m_game->getWasherRadius() + min(m_self->getWidth(), m_self->getHeight()) / 2.) {
				projectileAims[i + 1] = 1;
				curEnemyProjectileAimsCnt++;
			}
		}

		if (enemy.getDurability() - curEnemyProjectileAimsCnt * m_game->getWasherDamage() < 0.)
			isProjectileLowHPAim = true;

		pdd oiledPoint = { m_self->getX() - m_myMovingDirection * m_myMovingVector.first * (m_self->getHeight()/2. + m_game->getOilSlickRadius() + m_game->getOilSlickInitialRange()), 
						   m_self->getY() - m_myMovingDirection * m_myMovingVector.second * (m_self->getHeight() / 2. + m_game->getOilSlickRadius() + m_game->getOilSlickInitialRange()) };
		if (isPointOnRoad(oiledPoint) && distance({ enemy.getX(), enemy.getY() }, oiledPoint) < 2 * m_tileSize &&
			abs(enemy.getAngleTo(oiledPoint.first, oiledPoint.second)) < PI / 6.)
			oilAimsCnt++;

// 		int maxOilTiles = 5;
// 		for (int i = 2; i < 2 + maxOilTiles; ++i) {
// 			int j = (m_curRouteIndex - i + m_route.size()) % m_route.size();
// 			if (isCarInTile(enemy, m_route[j].pos))
// 				oilAimsCnt++;
// 		}
	}
	int projectileAimsCnt = projectileAims[0] + projectileAims[1] + projectileAims[2];
	if (projectileAimsCnt >= 2 ||
		(projectileAimsCnt >= 1 && isProjectileLowHPAim) ||
		(m_self->getProjectileCount() >= 4 && projectileAimsCnt >= 1))
		m_move->setThrowProjectile(true);
	if (oilAimsCnt >= 1)
		m_move->setSpillOil(true);
}

void MyStrategy::updateValues() {
	m_myTile = getCarTile(*m_self);
	m_mySpeed = getCarSpeedModule(*m_self);
	m_myCarCorners = getCarCorners(*m_self);
	m_myPos = { m_self->getX(), m_self->getY() };
	m_myMovingDirection = getCarMovingDirection(*m_self);
	m_myMovingVector = { (m_myCarCorners[0].first - m_myCarCorners[3].first) / distance(m_myCarCorners[0], m_myCarCorners[3]),
						 (m_myCarCorners[0].second - m_myCarCorners[3].second) / distance(m_myCarCorners[0], m_myCarCorners[3]) };

	m_waypointsCnt = m_world->getWaypoints().size();
	m_nextWaypointPos = { m_self->getNextWaypointX(), m_self->getNextWaypointY() };

	for (int i = 0; i < m_world->getCars().size(); ++i)
		if (m_world->getCars()[i].getId() != m_self->getId())
			m_otherCarState[i] = toCarState(m_world->getCars()[i]);
	m_myCarState = toCarState(*m_self);

	m_myCarController.wheelTurn = m_self->getWheelTurn();
	m_myCarController.enginePower = m_self->getEnginePower();

	if (m_isMode2x2) {
		int flag = 0;
		for (int i = 0; i < m_world->getWidth(); ++i) {
			for (int j = 0; j < m_world->getHeight(); ++j)
				if (m_tiles[i][j] == TileType::UNKNOWN) {
					m_tiles[i][j] = m_world->getTilesXY()[i][j];
					flag = 1;
				}
		}
		m_isMode2x2 = flag;
	}
}

void MyStrategy::simulatePhysics(CarState &state, const Controller &newController) {
	pdd crossUnit = getCrossUnitVector(state.angle);
	pdd longUnit = getLongUnitVector(state.angle);
	
	int dir = sgn(newController.enginePower) >= 0 ? 0 : 1;
	pdd acceleration = longUnit * (newController.isBreaking ? 0. : 1.) * m_carAccelFactor[state.carType][dir] * newController.enginePower;

	state.angularSpeed -= state.baseAngSpeed;
	double baseAngSpeed = m_carRotFactor * newController.wheelTurn * dot(state.speed, longUnit);
	state.angularSpeed += baseAngSpeed;

	for (int i = 0; i < m_physIters; ++i) {
		state.pos += state.speed * m_physDt;
		state.speed += acceleration;
		state.speed *= m_frictMul;

		state.speed -= limit(dot(state.speed, longUnit), (newController.isBreaking ? m_crossFrict : m_longFrict)) * longUnit;
		state.speed -= limit(dot(state.speed, crossUnit), m_crossFrict) * crossUnit;

		state.angle += state.angularSpeed * m_physDt;
		crossUnit = getCrossUnitVector(state.angle);
		longUnit = getLongUnitVector(state.angle);

// 		state.angSpeed -= limit(state.angSpeed, m_carRotFactor);
		state.angularSpeed = baseAngSpeed + (state.angularSpeed - baseAngSpeed) * m_rotFrictMul;
	}
	state.baseAngSpeed = baseAngSpeed;
	state.controller = newController;
	normalizeAngle(state.angle);
}

void MyStrategy::computeStates(const CarState& base, const int baseRouteIndex) {
	const int stepsCnt = min(1+int(length(base.speed)/10.), 3);
 	const int k[3] = {50, 50, 20};
	const double wheelStep[3] = {0.1, 0.2, 0.};

	m_myStates = computeStatesAux(base, baseRouteIndex, 0, stepsCnt, k, wheelStep).second;
	m_myStateIndex = 0;
	m_isNeededStatesUpdate = false;
}

pair<double, vector<CarState>> MyStrategy::computeStatesAux(const CarState& base, const int baseRouteIndex, const int step, const int stepsCnt, const int* k, const double* wheelStep) {
	if (step >= stepsCnt)
		return make_pair(0., vector<CarState>());

	pair<double, vector<CarState>> best;
	vector<CarState> curStates;

	for (int isBreaking = 0; isBreaking <= 0; ++isBreaking) {
		double _l = max(-1., base.controller.wheelTurn - k[step] * m_wheelCPT);
		double _r = min(1., base.controller.wheelTurn + k[step] * m_wheelCPT);
// 		while (abs(_r - _l) > wheelStep[step]) {
// 			double _m[2] = { _l + (_r - _l) / 3., _l + 2. * (_r - _l) / 3. };
// 			int _t = -1;
// 			double _p = -1.;
			
// 			for (int i = 0; i < 2; ++i) {
			for (double wheelTurn = _l; wheelTurn <= _r; wheelTurn += max(0.05, wheelStep[step])) {
// 				double wheelTurn = _m[i];
				curStates.clear();
				curStates.push_back(base);

				double curPts = 0.;
				CarState cur = base;
				vector<pdd> curCorners;
				int curRouteIndex = baseRouteIndex;
				pii curTile = getTile(cur.pos);

				Controller dest(wheelTurn, 1., isBreaking, cur.controller.useNitro);

				bool collision[2] = { 0 };
				bool lostRoute = false;
				int tick = 0;
				for (; tick < k[step]; ++tick) {
					simulatePhysics(cur, getUpdatedController(cur.controller, dest));
					curStates.push_back(cur);

					curCorners = getCorners(cur.pos, m_game->getCarWidth(), m_game->getCarHeight(), cur.angle);
					for (int j = 0; !collision[0] && !collision[1] && j < curCorners.size(); ++j) {
						if (!isPointOnRoad(curCorners[j]))
							collision[j / 2] = true;
						for (int k = 0; k < m_world->getCars().size(); ++k)
						if (m_world->getCars()[k].getId() != m_self->getId() && isPointInPolygon(getCarCorners(m_world->getCars()[k]), curCorners[j]))
							collision[j / 2] = true;
					}
					if (collision[0] || collision[1])
						break;

					curTile = getTile(cur.pos);
					if (curTile == m_route[curRouteIndex + 1].pos)
						curRouteIndex++;
					else if (curTile != m_route[curRouteIndex].pos) {
						lostRoute = true;
						break;
					}
					double a = getAngle(m_route[curRouteIndex + 1].pos - m_route[curRouteIndex].pos);
					pdd l = getLongUnitVector(a);
					pdd c = getCrossUnitVector(a);
					pdd v1 = (m_routeRealXY[curRouteIndex + 1] - cur.pos);
					v1 /= length(v1);
					double angleToWaypoint = getAngleBetween(v1, getLongUnitVector(cur.angle));
					// 					if (angleToWaypoint > PI)
					// 						angleToWaypoint -= DOUBLE_PI;

					curPts += 1. + dot(v1, cur.speed)/20. - angleToWaypoint;// * (DOUBLE_PI - abs(wheelTurn - angleToWaypoint));
				}

				if (!lostRoute) {
// 					debugPoint(curCorners[0], 255, 0, 0);
// 					debugPoint(curCorners[1], 0, 255, 0);
// 					debugPoint(curCorners[2], 0, 0, 255);
// 					debugPoint(curCorners[3], 0, 0, 0);

					if (collision[0] || collision[1])
						curPts *= (collision[0]) ? 0.2 : 0.5;
					else {
						pair<double, vector<CarState>> sts = computeStatesAux(cur, curRouteIndex, step + 1, stepsCnt, k, wheelStep);
						curPts += sts.first;
						for (int i = 0; i < sts.second.size(); ++i)
							curStates.push_back(sts.second[i]);
					}
				}
				else
					curPts *= 0.001;
				    
// 				if (curPts > _p) {
// 					_p = curPts;
// 					_t = i;
// 				}
				if (curPts > best.first) {
// 					debugPoint(curCorners[0], 255, 0, 0);
// 					debugPoint(curCorners[1], 0, 255, 0);
// 					debugPoint(curCorners[2], 0, 0, 255);
// 					debugPoint(curCorners[3], 0, 0, 0);
					best.first = curPts;
					best.second = curStates;
				}
			}

// 			if (_t == 0)
// 				_r = _m[0];
// 			else if (_t == 1)
// 				_l = _m[1];
// 			else break;
	
	}

	return best;
}

Controller MyStrategy::getUpdatedController(const Controller& old, const Controller& dest) {
	Controller ans(old);
	if (abs(old.wheelTurn - dest.wheelTurn) + EPS >= m_game->getCarWheelTurnChangePerTick())
		ans.wheelTurn += sgn(dest.wheelTurn - old.wheelTurn) * m_game->getCarWheelTurnChangePerTick();
	if (abs(old.enginePower - dest.enginePower) + EPS >= m_game->getCarEnginePowerChangePerTick())
		ans.enginePower += sgn(dest.enginePower - old.enginePower) * m_game->getCarEnginePowerChangePerTick();
	return ans;
}

void MyStrategy::debugPoint(const pdd& p, int r, int g, int b) {
#ifdef DEBUG
	printf("(%.2lf, %.2lf) rgb(%d, %d, %d)\n", p.first, p.second, r, g, b);
#endif // DEBUG
}

void MyStrategy::debugLine(const pdd& st, const pdd& en, int r, int g, int b) {
#ifdef DEBUG
	printf("(%.2lf, %.2lf) (%.2lf, %.2lf) rgb(%d, %d, %d)\n", st.first, st.second, en.first, en.second, r, g, b);
#endif // DEBUG
}

void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
	m_self = &self;
	m_world = &world;
	m_game = &game;
	m_move = &move;

#ifdef DEBUG
	freopen("C:\\Users\\ORLEON\\Desktop\\local-runner\\plugins\\debug1.txt", "w", stdout);
#endif // DEBUG
// 	freopen("C:\\Users\\ORLEON\\Desktop\\debug1.txt", "a+", stdout);

	if (!m_isInitialized)
		init();
	else
		updateValues();
	m_move->setEnginePower(1.);

// 	double m = m_game->getBuggyMass();
// 	double m1 = m_game->getCarLengthwiseMovementFrictionFactor();
// 	double m2 = m_game->getCarCrosswiseMovementFrictionFactor();
// 	double kw = m_game->getCarWheelTurnChangePerTick();
// 	double ke = m_game->getCarEnginePowerChangePerTick();
// 	double kef = m_game->getBuggyEngineForwardPower();
// 	double ker = m_game->getBuggyEngineRearPower();
// 	double mr = m_game->getCarRotationFrictionFactor();
// 	double mra = m_game->getCarRotationAirFrictionFactor();
// 	m_self->getEnginePower();
// 	int k = 8;
// 	for (int i = 0; i < k * m_world->getWidth(); ++i) {
// 		for (int j = 0; j < k * m_world->getHeight(); ++j) {
// 			pdd p = { double(i) * m_game->getTrackTileSize() / double(k), double(j) * m_game->getTrackTileSize() / double(k) };
// 			if (isPointOnRoad(p))
// 				debugPoint(p);
// 		}
// 	}
// 	debugPoint(m_myCarCorners[0], 255, 0, 0);
// 	debugPoint(m_myCarCorners[1], 0, 255, 0);
// 	debugPoint(m_myCarCorners[2], 0, 0, 255);
// 	debugPoint(m_myCarCorners[3], 0, 0, 0);
	debugPoint(m_myPos + make_pair(m_self->getSpeedX(), m_self->getSpeedY()) * 10.);


	if (m_world->getTick() > m_game->getInitialFreezeDurationTicks()) {
// 		CarState cur = m_myCarState;
// 		for (int i = 0; i < 50; ++i) {
// 			simulatePhysics(cur, m_myCarController);
// 			vector<pdd> corners = getCorners(cur.pos, m_game->getCarWidth(), m_game->getCarHeight(), cur.angle);
// 			debugPoint(corners[0], 255, 0, 0);
// 			debugPoint(corners[1], 0, 255, 0);
// 			debugPoint(corners[2], 0, 0, 255);
// 			debugPoint(corners[3], 0, 0, 0);
// 		}
// 		CarState cur = toCarState(self);
// 		simulatePhysics(m_predictedState, m_myCarController);
// 		if (abs(cur.speed.first - m_predictedState.speed.first) > 1e-5)
// 			printf("Shit!!!\n");
// 		printf("Tick %d:\n", m_world->getTick());
// 		m_predictedState.print();
// 		cur.print();

// 		double a = 0., b = 0.05, l = 0., r = 30., k = (a - b) / (a - r), m = b - k*r;
// 		double t = k*m_mySpeed + m;

// 		double sx = m_self->getSpeedX();
// 		double sy = m_self->getSpeedY();
// 		double m = m_self->getMass();
// 		double ep = m_game->getJeepEngineForwardPower();
// 		double mff = m_game->getCarMovementAirFrictionFactor();
// 		double sa = m_self->getAngularSpeed();
// // 		m_move->setEnginePower(0.25);
// // 		if (m_world->getTick() > m_game->getInitialFreezeDurationTicks() + 40)
// // 			m_move->setUseNitro(true);
// // 		double ep = m_self->getEnginePower();
// 
// 		printf("%lf\n", abs(sa));
// 
// 		return;
		if (m_myLastTile != m_myTile)
			updateRoute(m_route[m_curRouteIndex].direction, m_route[m_nextRouteIndex].direction);
//  	if (isCarInTile(*m_self, m_lastWaypointPos)) {
// 			updateRoute(m_route[m_curRouteIndex].direction);
// 		} else if (isCarInTile(*m_self, m_route[m_nextRouteIndex].pos)) {
// 			m_curRouteIndex = m_nextRouteIndex;
// 			m_nextRouteIndex = (m_nextRouteIndex + 1) % m_route.size();
// 		} else if (!isCarInTile(*m_self, m_route[m_curRouteIndex].pos)) {
// 			updateRoute();
// 		}

		debugLine(m_myPos, m_routeRealXY[m_nextRouteIndex], 0, 255, 0);

		if (isStuck())
			setMode(Mode::STUCK);
		if (m_self->getDurability() < EPS)
			setMode(Mode::NORMAL, true);

		if (m_activeMode == Mode::NORMAL) {
			normalMode();
			fireController();
			if (m_self->getDurability() < 0.3) {
//				lowhpMode();
			}
// 			else
// 				nitroController();
		}
		else if (m_activeMode == Mode::STUCK) {
			debugPoint(m_myPos, 255, 0, 0);
			stuckMode();
			fireController();
		}
		m_modeTicks++;
	}

	if (move.isBrake()) {
		debugPoint(m_myCarCorners[2], 255, 0, 0);
		debugPoint(m_myCarCorners[3], 255, 0, 0);	
	}

// 	fclose(stdout);
#ifdef DEBUG
	fclose(stdout);
	remove("C:\\Users\\ORLEON\\Desktop\\local-runner\\plugins\\debug.txt");
	rename("C:\\Users\\ORLEON\\Desktop\\local-runner\\plugins\\debug1.txt", "C:\\Users\\ORLEON\\Desktop\\local-runner\\plugins\\debug.txt");
#endif

	m_myLastPos = m_myPos;
	m_myLastTile = m_myTile;
	m_myCarController.isBreaking = m_move->isBrake();
	m_myCarController.useNitro = m_move->isUseNitro();
	m_myStateIndex++;
}