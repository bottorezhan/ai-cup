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

	double distance = -1;
	Puck cp;

	for (const auto &p : plist)
	{
		double dist = (p.pos - pos).mag();

		if (distance == -1 || dist < distance)
		{
			distance = dist;
			cp = p;
		}
	}

	return {distance, cp};
}

Vector2D bully_tactic(Bumper &bumper, const GameState &state)
{
	Vector2D force(0, 0);

	const Sled &enemy_sled = state.en_sled;
	Vector2D target = enemy_sled.pos;

	// if (!enemy_sled.path.empty()) {
	// 	target = enemy_sled.pos;
	// 	Vector2D center(0, 0);
	// 	double radius;
	// 	tie(center, radius) = find_center(enemy_sled.path);

	// 	if (radius > 100) {
	// 		center = enemy_sled.pos;
	// 	}

	// 	target = center;
	// 	cerr << "Circle radius: " << radius << " coords: " << center.x << ' ' << center.y << '\n';
	// }


	cerr << bumper.last_target << '\n';

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
		bool res = run_to(bumper.pos, bumper.vel, target, force, enemy_sled.dir);
		cerr << res << '\n';
		// run_to(bumper.pos, bumper.vel, target, force, enemy_sled.dir);

		// force = (target - bumper.pos).limit(BUMPER_FORCE_LIMIT);
		bumper.last_target = closest_puck.index;
		
		// return force;
	} else {
		bumper.last_target = -1;
	}

	force = (target - bumper.pos).limit(BUMPER_FORCE_LIMIT);
	return force;
}

int main()
{
	GameState state;

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
				// force = bully_tactic(bumper, state);
			}

			// cerr << force.x << " " << force.y << "\n";
			cout << force.x << " " << force.y << " ";
		}

		const auto& sled = state.my_sled;

		// cancel 

		// Just keep making circles and march across the playing field.
		double sledDir = 0;
		int loopSize = 40;
		int loopGap = 5;
		int startup = 12;

		// Make the sled drive around in circles.
		if (turnNum < startup)
		{
			// Initially, try to move off-center a little bit.
			if (turnNum < 6)
				sledDir = -0.2;
			else
				sledDir = 0.2;
		}
		else if ((turnNum - startup) % (loopSize + loopGap) < loopSize)
		{
			// Drive in a loop most of the time.
			sledDir = 2 * acos(-1.0) / loopSize;
		}
		else
		{
			// Move the loop ahead.
			sledDir = 0;
		}

		// Output the sled's move.

		cout << sledDir << endl;

		cin >> turnNum;
	}
}
