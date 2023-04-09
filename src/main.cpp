// A sample player implemented in C++.  The sled just marches across
// the middle trying to capture pucks.  The bumpers try to push the
// pucks toward the center so they can be captured.
//
// ICPC Challenge
// Sturgill, Baylor University

#include "Util.h"
#include <vector>
#include <iostream>
#include <list>
#include <tuple>
#include <cassert>
#include <queue>
#include <cmath>
#include <algorithm>

using namespace std;

/** Simple representation for a puck. */
struct Puck
{
	// Position of the puck.
	Vector2D pos;

	// Puck velocity
	Vector2D vel;

	// Puck color
	int color;

	int index;
};

/** Simple representation for a bumper. */
struct Bumper
{
	// Position of the bumper.
	Vector2D pos;

	// Bumper velocity
	Vector2D vel;

	int last_target;

	int index = -1;
};

/** Simple representation for a sled. */
struct Sled
{
	// Position of the sled.
	Vector2D pos;

	vector<Vector2D> path;

	// Sled direction.
	double dir;
};

struct GameState
{

public:
	vector<Puck> my_pucks, en_pucks, nu_pucks, all_pucks;
	vector<Bumper> my_blist, en_blist;
	Sled my_sled, en_sled;

	void read()
	{
		int n;

		// Read all the puck locations.
		cin >> n;
		my_pucks.resize(0);
		en_pucks.resize(0);
		nu_pucks.resize(0);
		all_pucks.resize(n);

		for (int i = 0; i < n; i++)
		{
			Puck puck;
			cin >> puck.pos.x >> puck.pos.y >> puck.vel.x >> puck.vel.y >> puck.color;
			puck.index = i;

			all_pucks[i] = puck;

			if (puck.color == 0)
			{
				my_pucks.push_back(puck);
			}
			else if (puck.color == 1)
			{
				en_pucks.push_back(puck);
			}
			else
			{
				nu_pucks.push_back(puck);
			}
		}

		// Read all the bumper locations.
		cin >> n;
		assert(n == 4);
		my_blist.resize(2);
		en_blist.resize(2);

		for (int i = 0; i < n; i++)
		{
			Bumper &bumper = (i < 2 ? my_blist[i] : en_blist[i - 2]);
			cin >> bumper.pos.x >> bumper.pos.y >> bumper.vel.x >> bumper.vel.y;

			bumper.index = i;
		}

		// Read all the sled locations.
		cin >> n;
		assert(n == 2);

		for (int i = 0; i < n; i++)
		{
			Sled &sled = (i == 0 ? my_sled : en_sled);
			cin >> sled.pos.x >> sled.pos.y >> sled.dir;

			// Just ignore the path history for this sled.
			int m;
			cin >> m;

			sled.path.resize(m);
			for (int j = 0; j < m; j++)
			{
				cin >> sled.path[j].x;
				cin >> sled.path[j].y;
			}
		}
	}
};

pair<Vector2D, double> circumscribed_circle(Vector2D p1, Vector2D p2, Vector2D p3)
{
	double a = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
	double b = sqrt((p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y));
	double c = sqrt((p3.x - p2.x) * (p3.x - p2.x) + (p3.y - p2.y) * (p3.y - p2.y));
	double radius = (a * b * c) / (sqrt((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c)));
	double d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));
	double xp = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) + (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) + (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;
	double yp = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) + (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) + (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;

	return {{xp, yp}, radius};
}
/** Return a copy of x that's constrained to be between low and high. */
double clamp(double x, double low, double high)
{
	if (x < low)
		x = low;
	if (x > high)
		x = high;
	return x;
}

/** Compute a force vector that can be applied to a bumper to get it
	to run through the given target location.  Pos and vel are the
	bumper's current position and velocity.  Target is the position we
	want to run through and force is a returned vector that will move
	the bumper toward the target.  The function returns true until it
	looks like the next move will take us through the target
	location.  */
bool run_to(Vector2D const &pos,
			Vector2D const &vel,
			Vector2D const &target,
			Vector2D &force,
			double epsilon = 0.1)
{
	// Compute a vector that will move self toward the target point.
	Vector2D direction = target - pos;
	double dist = direction.mag();

	// How much force do we have left to use in the force vector.
	double resForce = BUMPER_FORCE_LIMIT - force.mag();

	// Normalize the direction we need to move.
	direction = direction.norm();

	// First, cancel out any movement that is perpendicular to the desired
	// movement direction.
	Vector2D perp = Vector2D(-direction.y, direction.x);
	force = -(perp * vel) * perp;

	// Use all the residual force to move toward the target.
	resForce = BUMPER_FORCE_LIMIT - force.mag();
	force = force + direction.limit(resForce);

	// See if this move will cross close enough to the target location.
	Vector2D nvel = (vel + force).limit(BUMPER_SPEED_LIMIT);
	double t = clamp((target - pos) * nvel / (nvel * nvel), 0, 1);
	if ((pos + t * nvel - target).mag() < epsilon)
		return true;

	return false;
}

pair<Vector2D, double> find_center(vector<Vector2D> path)
{

	if (path.size() < 3)
	{
		return {(path.front() - path.back()) * 0.5, 0};
	}

	return circumscribed_circle(path.front(), path[path.size() / double(2)], path.back());
}

pair<double, Puck> get_closest_puck(const vector<Puck> &plist, Vector2D pos)
{

	Puck cp = *min_element(plist.begin(), plist.end(), [&](const auto& a, const auto& b) {
		return ((a.pos - pos).mag()) < ((b.pos - pos).mag());
	});
	double distance = (cp.pos - pos).mag();

	return {distance, cp};
}

Vector2D bully_tactic(Bumper &bumper, const GameState &state)
{
	Vector2D force(0, 0);

	const Sled &enemy_sled = state.en_sled;
	Vector2D target = enemy_sled.pos;

	bool target_existed = false;
	Puck closest_puck;
	double distance_to_closest_puck;

	if (bumper.last_target != -1 && (state.all_pucks[bumper.last_target].pos - enemy_sled.pos).mag() < 200) {
		closest_puck = state.all_pucks[bumper.last_target];
		distance_to_closest_puck = (state.all_pucks[bumper.last_target].pos - enemy_sled.pos).mag();
		target_existed = true;
	} else {
		tie(distance_to_closest_puck, closest_puck) = get_closest_puck(state.en_pucks, enemy_sled.pos);
	}

	if ((!target_existed and distance_to_closest_puck < 100) or (target_existed and distance_to_closest_puck < 200))
	{
		target = closest_puck.pos;
		bool res = run_to(bumper.pos, bumper.vel, target, force);
		bumper.last_target = closest_puck.index;
		return force;
	} else {
		bumper.last_target = -1;
	}

	force = (target - bumper.pos).limit(BUMPER_FORCE_LIMIT);
	// bool res = run_to(bumper.pos, bumper.vel, target, force);
	return force;
}

Vector2D support_tactic(Bumper &bumper, const GameState &state)
{
	Vector2D force(0, 0);
	Vector2D target = CENTER;

	double mrl, mrr;

	if (state.my_sled.pos.x < 200) {
		mrl = 0;
		mrr = 200;
	} else {
		mrl = 600;
		mrr = 800;
	}

	for (auto p : state.en_pucks) {

		if (mrl <= p.pos.x && p.pos.x <= mrr) {
			target = p.pos;
			bool res = run_to(bumper.pos, bumper.vel, target, force);
			return force;
		}
	}

	bool target_existed = (bumper.last_target != -1);
	Puck closest_puck;
	double distance_to_closest_puck;

	if (target_existed) {
		Vector2D last_target = state.all_pucks[bumper.last_target].pos;
		double distance_to_center = (state.all_pucks[bumper.last_target].pos - CENTER).mag();

		if (distance_to_center < 300) {
			target = last_target;
		} else {
			bumper.last_target = -1;
		}
	}
	
	if (bumper.last_target == -1) {
		vector<Puck> plist;

		plist.insert(plist.end(), state.my_pucks.begin(),state.my_pucks.end());
		plist.insert(plist.end(), state.nu_pucks.begin(),state.nu_pucks.end());

		tie(distance_to_closest_puck, closest_puck) = get_closest_puck(plist, CENTER);
		target = closest_puck.pos;
		bumper.last_target = closest_puck.index;
	}
	
	// force = (target - bumper.pos).limit(BUMPER_FORCE_LIMIT);
	bool res = run_to(bumper.pos, bumper.vel, target, force);
	return force;
}

void make_turn_rad(queue<double>& sled_moves, double rad, double ang_limit = SLED_TURN_LIMIT) {
	int nu_moves = ceil(fabs(rad) / min(ang_limit, SLED_TURN_LIMIT));
	double moves_rad = rad / nu_moves;

	while (nu_moves--) {
		sled_moves.push(moves_rad);
	}
}

void make_turn_deg(queue<double>& sled_moves, double derg) {
	double rad = derg / 180 * M_PI;
	make_turn_rad(sled_moves, rad);
}

void move_to_point(queue<double>& moves, Sled sled, Vector2D target) {
	cerr << target.x << ' ' << target.y << '\n';
	int nunu = sled.dir / (M_PI * 2);
	double dir = sled.dir - nunu * (M_PI * 2);

	Vector2D dif = target - sled.pos;
	double dif_ang = atan2(dif.y, dif.x);
	double res = (dif_ang - dir);

	// cerr << res << ' ' << dir << ' ' << dif_ang << '\n';

	res = max(min(res, 0.5), -0.5);

	double dist = dif.mag();

	moves.push(res);
}

int main()
{
	GameState state;
	queue<double> sled_moves;
	sled_moves.push(0);
	sled_moves.push(0);
	sled_moves.push(0);
	bool move_type = false;

	int n, turnNum;
	cin >> turnNum;

	while (turnNum >= 0)
	{
		// Read game state
		state.read();

		const bool ENEMY_SLED = 1;

		// Choose a move for each sled.
		for (int i = 0; i < 2; i++)
		{
			Bumper &bumper = state.my_blist[i];
			Vector2D force(0, 0);
			bool is_bully = (i == 1);

			if (is_bully)
			{
				force = bully_tactic(bumper, state);
			}
			else
			{
				force = support_tactic(bumper, state);
			}

			// cerr << force.x << " " << force.y << "\n";
			cout << force.x << " " << force.y << " ";
		}

		const auto& sled = state.my_sled;

		// Puck clos_puck;
		// double clos_dist;

		
		// tie(clos_dist, clos_puck) = get_closest_puck(state.en_pucks, sled.pos);

		// if (clos_dist < 10) {
		// 	make_turn_rad(sled_moves, M_PI_2);
		// 	make_turn_rad(sled_moves, M_PI * 2, 0.2);
		// } else {
		// 	move_to_point(sled_moves, sled, clos_puck.pos);
		// }

		if (sled_moves.empty()) {
			make_turn_rad(sled_moves, M_PI_2);
			sled_moves.push(0);
			sled_moves.push(0);
			sled_moves.push(0);
			sled_moves.push(0);
			make_turn_rad(sled_moves, M_PI_2);
			sled_moves.push(0);
			sled_moves.push(0);
			sled_moves.push(0);
			sled_moves.push(0);
			make_turn_rad(sled_moves, M_PI_2);
			sled_moves.push(0);
			sled_moves.push(0);
			make_turn_rad(sled_moves, M_PI_2);
			sled_moves.push(0);
			sled_moves.push(0);
			sled_moves.push(0);
			sled_moves.push(0);
		}
		
		
		cout << sled_moves.front() << endl;
		sled_moves.pop();

		cin >> turnNum;
	}
}
