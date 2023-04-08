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
};

/** Simple representation for a bumper. */
struct Bumper
{
	// Position of the bumper.
	Vector2D pos;

	// Bumper velocity
	Vector2D vel;
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

/** Return true if a puck at position pos is the responsibility of the
	bumper with index bdex. */
bool my_side(int bdex, Vector2D pos)
{
	if (bdex == 0 && pos.y < 330)
		return true;
	if (bdex == 1 && pos.y > 470)
		return true;
	return false;
}

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

pair<Vector2D, double> find_center(vector<Vector2D> path)
{
	
	if (path.size() < 3) {
		return {(path.front() - path.back()) * 0.5, 0};
	}

	return circumscribed_circle(path.front(), path[path.size() / double(2)], path.back());
}

struct GameState
{
	vector<Puck> plist;
	vector<Bumper> blist;
	vector<Sled> slist;

	void read() {
		int n;

		// Read all the puck locations.
		cin >> n;
		plist.resize(n);
		for (int i = 0; i < n; i++)
		{
			Puck &puck = plist[i];
			cin >> puck.pos.x >> puck.pos.y >> puck.vel.x >> puck.vel.y >> puck.color;
		}

		// Read all the bumper locations.
		cin >> n;
		blist.resize(n);
		for (int i = 0; i < n; i++)
		{
			Bumper &bumper = blist[i];
			cin >> bumper.pos.x >> bumper.pos.y >> bumper.vel.x >> bumper.vel.y;
		}

		// Read all the sled locations.
		cin >> n;
		slist.resize(n);
		for (int i = 0; i < n; i++)
		{
			Sled &sled = slist[i];
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

Vector2D bully_tactic(const Bumper& bumper, const GameState& state) {
	Vector2D force(0, 0);

	int ENEMY_SLED = 1;
	Vector2D target(400, 400);
	
	if (state.slist[ENEMY_SLED].path.empty()) {
		target = state.slist[ENEMY_SLED].pos;
	} else {
		auto center = find_center(state.slist[ENEMY_SLED].path);

		if (center.second > 100) {
			center.first = state.slist[ENEMY_SLED].pos;
		}

		target = center.first;
	}

	run_to(bumper.pos, bumper.vel, target, force);

	// force = (target - bumper.pos).limit(BUMPER_FORCE_LIMIT);

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
			Bumper &bumper = state.blist[i];
			Vector2D force(0, 0);
			bool is_bully = (i == 1);

			if (is_bully) {
				force = bully_tactic(bumper, state);
			} else {
				// force = bully_tactic(bumper, state);
			}

			cout << force.x << " " << force.y << " ";
		}

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
