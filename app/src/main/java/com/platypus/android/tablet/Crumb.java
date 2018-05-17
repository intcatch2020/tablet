package com.platypus.android.tablet;

import org.jscience.geography.coordinates.UTM;


/**
 * A breadcrumb is a location, a unique identifier, and some data relevant to A* path planning
 */
public class Crumb
{
		// Instance fields
		long index;
		UTM location;
		double g = 0.; // initialize with zero
		double h; // run-time initialize e.g. set to the distance to the goal (if using A*)
		long parent;

		// Instance methods
		public Crumb(long _index, UTM _location)
		{
				index = _index;
				location = _location;
		}
		public void setG(double _g) { g = _g; }
		public double getG() { return g; }
		public void setH(double _h) { h = _h; }
		public void setParent(long _parent) { parent = _parent; }
		public long getParent() { return parent; }
		public double getCost() { return g + h; }
		public long getIndex() { return index; }
		public UTM getLocation() { return location; }
}
