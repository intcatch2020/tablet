package com.platypus.android.tablet.Path;

import android.util.Log;
import android.util.Pair;

import com.mapbox.mapboxsdk.geometry.LatLng;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

/**
 * Created by jason on 11/20/17.
 */

public class Region
{

		class Intersection
		{
				int first;
				int second;
				Double[] p;
				boolean usable;
				double distance_to_center;
				Intersection(int first_, int second_, Double[] p_)
				{
						first = first_;
						second = second_;
						p = p_.clone();
						usable = true;
						distance_to_center = distanceToCentroid(p);
				}
		}

		class Line
		{
				int index;
				Double[] p1;
				Double[] p2;
				Line(int index_, Double[] p1_, Double[] p2_)
				{
						index = index_;
						p1 = p1_.clone();
						p2 = p2_.clone();
				}

				Intersection findIntersection(Line j)
				{
						Double[] d1 = difference(p1, p2);
						Double[] d2 = difference(j.p1, j.p2);
						Double[] dp = difference(j.p1, p1);
						Double[] d1p = new Double[]{-d1[1], d1[0]};
						double denominator = dot(d1p, d2);
						double numerator = dot(d1p, dp);
						double c = numerator/denominator;
						return new Intersection(index, j.index, new Double[] {c*d2[0] + j.p1[0], c*d2[1] + j.p1[1]});
				}

				Intersection findIntersectionSegments(Line j)
				{
						// treat both lines as segments - only return an intersection if both segments actually contain the point
						Intersection intersection = findIntersection(j);

						// the sign of dot products will tell you if a point is on a line segment
						// vector from intersection to line segment end points
						// if the point is on the segment, the vectors will point in opposite directions (negative dot)
						// if the point is not on the segment, the vectors will point in the same direction (positive dot)

						// this line
						if (Math.signum(dot(difference(intersection.p, p1), difference(intersection.p, p2))) > 0) return null;

						// line j
						if (Math.signum(dot(difference(intersection.p, j.p1), difference(intersection.p, j.p2))) > 0) return null;

						return intersection;
				}

				double cross(Line j)
				{
						Double[] d1 = difference(p1, p2);
						Double[] d2 = difference(j.p1, j.p2);
						return d1[0]*d2[1] - d2[0]*d1[1];
				}

				double angle()
				{
						Double[] diff = difference(p1, p2);
						return Math.atan2(diff[1], diff[0]);
				}

				double length()
				{
						return distance(p1, p2);
				}

				double incidentAngle(Line j)
				{
						return wrapToPi(angle()-j.angle());
				}
		}

		class IntersectionMap extends HashMap<Pair<Integer, Integer>, Intersection>
		{
				// Override get and containsKey so that the integers can be in reverse order
				@Override
				public Intersection get(Object key_)
				{
						Pair<Integer, Integer> key = (Pair<Integer, Integer>) key_;
						Pair<Integer, Integer> reverseKey = new Pair<>(key.second, key.first);
						if (super.containsKey(key))
						{
								return super.get(key);
						}
						else if (super.containsKey(reverseKey))
						{
								return super.get(reverseKey);
						}
						return null;
				}

				@Override
				public boolean containsKey(Object key_)
				{
						Pair<Integer, Integer> key = (Pair<Integer, Integer>) key_;
						Pair<Integer, Integer> reverseKey = new Pair<>(key.second, key.first);
						if (super.containsKey(key))
						{
								return true;
						}
						else if (super.containsKey(reverseKey))
						{
								return true;
						}
						return false;
				}

				private IntersectionMap sliceBySharedKey(int i, int j)
				{
						// provide a list of all intersections that share only 1 of i or j, but not both!
						IntersectionMap result = new IntersectionMap();
						for (Entry<Pair<Integer, Integer>, Intersection> entry : this.entrySet())
						{
								Pair<Integer, Integer> key = entry.getKey();
								if ((key.first == i ^ key.second == j) || (key.second == i ^ key.first == j))
								{
										result.put(key, entry.getValue());
								}
						}
						return result;
				}

				IntersectionMap sliceBySharedKey(Intersection i)
				{
						return sliceBySharedKey(i.first, i.second);
				}
		}

		boolean approxEq(double a, double b, double tolerance)
		{
				return Math.abs(b-a) <= tolerance;
		}

		private ArrayList<LatLng> original_points = new ArrayList<>();
		private ArrayList<Double[]> convex_xy = new ArrayList<>();
		private ArrayList<Double[]> path_xy = new ArrayList<>();
		private ArrayList<LatLng> path_points = new ArrayList<>();
		private Double[] utm_centroid = new Double[2];
		private Double[] local_centroid = new Double[2];
		private UTM original_utm;
		private AreaType area_type;
		private double transect_distance = 10;
		private double max_dist_to_centroid = -1;
		private String logTag = "Region";

		public Region (ArrayList<LatLng> _original_points, AreaType _area_type, double _transect_distance) throws Exception
		{
				original_points = (ArrayList<LatLng>)_original_points.clone();
				convexHull();
				area_type = _area_type;
				transect_distance = _transect_distance;
				switch (area_type)
				{
						case SPIRAL:
								inwardNextHull(convex_xy, 0);
								break;

						case LAWNMOWER:
								lawnMower(convex_xy);
								break;

						default:
								Log.e(logTag, "Unknown region type");
								throw new Exception("Unknown region type");
				}
		}

		public Path convertToSimplePath()
		{
				// create a simple Path from path_points
				path_points.clear();
				for (Double[] p : path_xy)
				{
						Log.v(logTag, String.format("final sequence point = [%.1f, %.1f]", p[0], p[1]));
						// add back in the UTM offset
						p[0] += utm_centroid[0];
						p[1] += utm_centroid[1];
						LatLong latLong = UTM.utmToLatLong(UTM.valueOf(original_utm.longitudeZone(), original_utm.latitudeZone(), p[0], p[1], SI.METER), ReferenceEllipsoid.WGS84);
						path_points.add(new LatLng(latLong.latitudeValue(NonSI.DEGREE_ANGLE), latLong.longitudeValue(NonSI.DEGREE_ANGLE)));
				}
				return new Path(path_points);
		}

		private static double wrapToPi(double angle)
		{
				while (Math.abs(angle) > Math.PI)
				{
						angle -= 2*Math.PI*Math.signum(angle);
				}
				return angle;
		}

		private static <T> ArrayList<T> shiftArrayList(ArrayList<T> aL, int shift)
		{
				while (Math.abs(shift) > aL.size())
				{
						shift -= aL.size()*Math.signum(shift);
				}
				if (shift < 0)
				{
						shift = aL.size() + shift; // shifting backward is like shifting forward by more
				}
				Log.v("Region", String.format("Shifting forward by %d", shift));
				// https://stackoverflow.com/questions/29548488/shifting-in-arraylist
				ArrayList<T> aL_clone = (ArrayList<T>)aL.clone();
				if (aL.size() == 0)
						return aL_clone;

				T element = null;
				for(int i = 0; i < shift; i++)
				{
						// remove last element, add it to front of the ArrayList
						element = aL_clone.remove( aL_clone.size() - 1 );
						aL_clone.add(0, element);
				}

				return aL_clone;
		}

		private Double[] difference(Double[] a, Double[] b)
		{
				return new Double[]{b[0] - a[0], b[1] - a[1]};
		}

		private double distance(Double[] a, Double[] b)
		{
				Double[] diff = difference(a, b);
				return Math.sqrt(Math.pow(diff[0], 2.) + Math.pow(diff[1], 2.));
		}
		private double dot(Double[] a, Double[] b)
		{
				// a and b represent vector component magnitudes
				return a[0]*b[0] + a[1]*b[1];
		}

		private double dotNormalized(Double[] a, Double[] b)
		{
				// a and b represent vector component magnitudes
				double lengthA = Math.sqrt(Math.pow(a[0], 2.) + Math.pow(a[1], 2.));
				double lengthB = Math.sqrt(Math.pow(b[0], 2.) + Math.pow(b[1], 2.));
				return dot(a, b)/(lengthA*lengthB);
		}

		private double distanceToCentroid(Double[] a)
		{
				return distance(a, local_centroid);
		}

		private double[][] pairwiseDistances(ArrayList<Double[]> points)
		{
				double[][] result = new double[points.size()][points.size()];
				for (int i = 0; i < points.size(); i++)
				{
						for (int j = 0; j < points.size(); j++)
						{
								result[i][j] = distance(points.get(i), points.get(j));
						}
				}
				return result;
		}

		private int[] diameterPair(ArrayList<Double[]> points)
		{
				double[][] pairwise_distances = pairwiseDistances(points);
				double max_dist = -1.;
				int maxi = 0;
				int maxj = 0;
				for (int i = 0; i < points.size(); i++)
				{
						for (int j = 0; j < points.size(); j++)
						{
								if (pairwise_distances[i][j] > max_dist)
								{
										maxi = i;
										maxj = j;
										max_dist = pairwise_distances[i][j];
								}
						}
				}
				return new int[]{maxi, maxj};
		}

		private Boolean isInsideHull(ArrayList<Double[]> hull, Double[] p)
		{
				// assumes a hull, i.e. the order of points in the array matter!
				ArrayList<Double> normal_angles = lineSegmentNormalAngles(hull);
				Boolean isInside = true;
				for (int j = 0; j < hull.size(); j++)
				{
						Double[] normal = new Double[]{Math.cos(normal_angles.get(j)), Math.sin(normal_angles.get(j))};
						Double[] diff = difference(hull.get(j), p);
						if (dot(normal, diff) < 0)
						{
								isInside = false;
								break;
						}
				}
				return isInside;
		}

		private ArrayList<Boolean> isInsideHull(ArrayList<Double[]> hull, ArrayList<Double[]> points)
		{
				// Check sign of dot product between
				//     vector 1: vector from a vertex to the point in question
				//     vector 2: the normal vector associated with that vertex's line
				// If the dot product is negative, the point has to be outside of the hull
				ArrayList<Boolean> result = new ArrayList<>();
				ArrayList<Double> normal_angles = lineSegmentNormalAngles(hull);
				for (int i = 0; i < points.size(); i++)
				{
						boolean isInside = true;
						Double[] p = points.get(i);
						//Log.d(logTag, String.format("isInside(): checking point = [%.1f, %.1f]", p[0], p[1]));
						for (int j = 0; j < hull.size(); j++)
						{
								//Log.v(logTag, String.format("isInside(): hull vertex = [%.1f, %.1f]", hull.get(j)[0], hull.get(j)[1]));
								Double[] normal = new Double[]{Math.cos(normal_angles.get(j)), Math.sin(normal_angles.get(j))};
								Double[] diff = difference(hull.get(j), p);
								//Log.v(logTag, String.format("isInside(): normal = [%.1f, %.1f],  vertex to point = [%.1f, %.1f], dot = %.1f",
								//				normal[0], normal[1], diff[0], diff[1], dot(normal, diff)));
								if (dot(normal, diff) < 0)
								{
										Log.v(logTag, String.format("isInside(): point %d is not inside", i));
										isInside = false;
										break;
								}
						}
						result.add(isInside);
				}
				return result;
		}

		private Double[] calculateCentroid(ArrayList<Double[]> points)
		{
				Double[] result = new Double[]{0.0, 0.0};
				for (Double[] p : points)
				{
						result[0] += p[0]/points.size();
						result[1] += p[1]/points.size();
				}
				return result;
		}

		private ArrayList<Double> lineSegmentNormalAngles(ArrayList<Double[]> points)
		{
				ArrayList<Double> result = new ArrayList<>();
				for (int i = 0; i < points.size(); i++)
				{
						int j = (i+1) % points.size();
						Double[] a = points.get(i);
						Double[] b = points.get(j);
						double raw_angle = Math.atan2(b[1]-a[1], b[0]-a[0]) + Math.PI/2.0;
						result.add(wrapToPi(raw_angle));
				}
				return result;
		}

		private ArrayList<Double[]> unitNormalVector(ArrayList<Double[]> points)
		{
				ArrayList<Double> normal_angles = lineSegmentNormalAngles(points);
				ArrayList<Double[]> result = new ArrayList<>();
				for (int i = 0; i < points.size(); i++)
				{
						double angle = normal_angles.get(i);
						result.add(new Double[] {Math.cos(angle), Math.sin(angle)});
				}
				return result;
		}

		private void inwardNextHull(ArrayList<Double[]> previous_hull, int inward_count)
		{
				// recursive. Given the previous set of points, calculate the next set of points. Append them to path_xy until the centroid is reached.
				// the first points (i.e. path_xy is empty) are the first convex hull
				// the final point is the centroid
				Log.d(logTag, String.format("inwardNextHull called. path_xy has %d points", path_xy.size()));
				if (path_xy.size() < 1)
				{
						// first call: just add convex hull to path and recurse
						path_xy.addAll(previous_hull);
						//path_xy.add(path_xy.get(0).clone()); // close the hull
						inwardNextHull(previous_hull, 1);
						return; // recursion stack unravels here, so return, now with path_xy fully populated
				}


				// find the normal angles of the line segments (make sure it points inward)
				ArrayList<Double> normal_angles = lineSegmentNormalAngles(previous_hull);
				ArrayList<Line> lines = new ArrayList<>();

				// find the points if they were moved inward along their normal by the transect distance
				int N = previous_hull.size();
				for (int i = 0; i < N; i++)
				{
						Double[] p1 = previous_hull.get(i);
						int j = (i+1) % N; // wrap the index
						Double[] p2 = previous_hull.get(j);
						double angle = normal_angles.get(i);

						Log.v(logTag, String.format("Normal angle: %f", angle));
						Double[] new_1 = new Double[]{
										p1[0] + transect_distance*Math.cos(angle),
										p1[1] + transect_distance*Math.sin(angle)};
						Double[] new_2 = new Double[]{
										p2[0] + transect_distance*Math.cos(angle),
										p2[1] + transect_distance*Math.sin(angle)};
						lines.add(new Line(i, new_1, new_2));
				}

				// all possible intersections - do not include if they are not inside the hull!
				IntersectionMap intersections = new IntersectionMap();
				for (int i = 0; i < N; i++)
				{
						for (int j = 0; j < N; j++)
						{
								if (i != j)
								{
										Pair<Integer, Integer> ij = new Pair<>(i, j);
										if (!intersections.containsKey(ij))
										{
												Intersection intersection = lines.get(i).findIntersection(lines.get(j));
												if (isInsideHull(previous_hull, intersection.p)) intersections.put(ij, intersection);
										}
								}
						}
				}

				// there must be at least 3 intersections available, otherwise truncate
				if (intersections.size() < 3)
				{
						Log.i(logTag, "There are less than 3 possible vertices, truncating");
						path_xy.add(local_centroid);
						return;
				}

				// find intercept closest to center of hull
				ArrayList<Intersection> hull_intersections = new ArrayList<>();
				hull_intersections.add(null);
				double min_dist_to_centroid = 9999999;
				for (Intersection intersection : intersections.values())
				{
						if (intersection.usable
										&& intersection.distance_to_center < min_dist_to_centroid)
						{
								min_dist_to_centroid = intersection.distance_to_center;
								hull_intersections.set(0, intersection);
						}
				}

				// loop your way around, traveling on lines, always keeping a consistent clockwise motion
				int junk = 0;
				IntersectionMap next_possible_intersections;
				do
				{
						Intersection last_intersection = hull_intersections.get(hull_intersections.size()-1);
						// find all intercepts that share either i or j of current intercept
						next_possible_intersections = intersections.sliceBySharedKey(last_intersection);

						// trim that down so that we maintain a negative cross-product sign
						//      (i.e. if we started clockwise, do not consider intercepts that would be counterclockwise)
						Line line1, line2;
						double max_angle = -Math.PI;
						// need to use iterators so we can remove map entries while looping through the map
						for (Iterator<Map.Entry<Pair<Integer, Integer>, Intersection>> it =
						     next_possible_intersections.entrySet().iterator(); it.hasNext();)
						{
								Map.Entry<Pair<Integer, Integer>, Intersection> entry = it.next();
								if (hull_intersections.size() < 2)
								{
										// use centroid as first point
										line1 = new Line(-1, local_centroid, last_intersection.p);
								}
								else
								{
										line1 = new Line(-1, hull_intersections.get(hull_intersections.size()-2).p, last_intersection.p);
								}
								line2 = new Line(-1, last_intersection.p, entry.getValue().p);
								double cross = line1.cross(line2);
								if (cross >= -0.01)
								{
										it.remove();
										continue;
								}

								double angle = line1.incidentAngle(line2);
								if (angle > max_angle) max_angle = angle;
						}

						// also trim by angle
						for (Iterator<Map.Entry<Pair<Integer, Integer>, Intersection>> it =
						     next_possible_intersections.entrySet().iterator(); it.hasNext();)
						{
								Map.Entry<Pair<Integer, Integer>, Intersection> entry = it.next();

								if (hull_intersections.size() < 2)
								{
										// use centroid as first point
										line1 = new Line(-1, local_centroid, last_intersection.p);
								}
								else
								{
										line1 = new Line(-1, hull_intersections.get(hull_intersections.size()-2).p, last_intersection.p);
								}
								line2 = new Line(-1, last_intersection.p, entry.getValue().p);
								double angle = line1.incidentAngle(line2);
								if (!approxEq(angle, max_angle, 0.01))
								{
										it.remove();
								}
						}

						if (next_possible_intersections.size() < 1) break;

						// find the closest point and add it to hull
						double dist = 9999999;
						Intersection next_intersection = null;
						for (Intersection intersection : next_possible_intersections.values())
						{
								double possible_dist = distance(last_intersection.p, intersection.p);
								if (possible_dist < dist)
								{
										dist = possible_dist;
										next_intersection = intersection;
								}
						}

						if (hull_intersections.contains(next_intersection)) break; // next point has already been used

						hull_intersections.add(next_intersection);
				} while (true);

				ArrayList<Double[]> points = new ArrayList<>();
				for (Intersection intersection : hull_intersections)
				{
						points.add(intersection.p);
				}
				// there must be at least 3 intersections available, otherwise truncate
				if (points.size() < 3)
				{
						Log.i(logTag, "There are less than 3 possible vertices, truncating");
						path_xy.add(local_centroid);
						return;
				}

				ArrayList<Double[]> new_hull = convexHull(points);
				// OR if the new hull doesn't contain the previous center
				if (!isInsideHull(new_hull, local_centroid))
				{
						Log.i(logTag, "New hull does not contain previous center, truncating");
						path_xy.add(local_centroid);
						return;
				}

				local_centroid = calculateCentroid(new_hull);
				Log.i(logTag, String.format("New local centroid = [%.1f, %.1f]", local_centroid[0], local_centroid[1]));

				double dist = 9999999;
				int closest_index = 0;
				for (int i = 0; i < new_hull.size(); i++)
				{
						double possible_dist = distance(new_hull.get(i), previous_hull.get(previous_hull.size()-1));
						if (possible_dist < dist)
						{
								dist = possible_dist;
								closest_index = i;
						}
				}
				new_hull = shiftArrayList(new_hull, -(closest_index+1));

				path_xy.addAll(new_hull);
				//path_xy.add(new_hull.get(0).clone()); // close the hull before moving to inward hull

				inward_count += 1;
				inwardNextHull(new_hull, inward_count);
		}

		private void convexHull() throws Exception
		{
				// set convex_xy based on original_points
				ArrayList<Double[]> utm_points = latLngToUTM(original_points);
				ArrayList<Double[]> utm_hull = convexHull(utm_points);
				convex_xy = zeroCentroid(utm_hull);
				// Double[] should_be_zero_centroid = calculateCentroid(convex_xy);
		}

		private ArrayList<Double[]> convexHull(ArrayList<Double[]> points)
		{
				ArrayList<Integer> convex_indices = GrahamScan.getConvexHull(points);
				ArrayList<Double[]> hull = new ArrayList<>();
				for (Integer i : convex_indices)
				{
						hull.add(points.get(i));
				}
				return hull;
		}

		private ArrayList<Double[]> latLngToUTM(ArrayList<LatLng> points)
		{
				ArrayList<Double[]> result = new ArrayList<>();
				for (LatLng wp : points)
				{
						UTM utm = UTM.latLongToUtm(LatLong.valueOf(wp.getLatitude(), wp.getLongitude(), NonSI.DEGREE_ANGLE), ReferenceEllipsoid.WGS84);
						result.add(new Double[]{utm.eastingValue(SI.METER), utm.northingValue(SI.METER)});
				}
				original_utm = UTM.latLongToUtm(LatLong.valueOf(points.get(0).getLatitude(), points.get(0).getLongitude(), NonSI.DEGREE_ANGLE), ReferenceEllipsoid.WGS84);
				return result;
		}

		private ArrayList<Double[]> zeroCentroid(ArrayList<Double[]> points)
		{
				utm_centroid = calculateCentroid(points);
				local_centroid = new Double[]{0., 0.};
				ArrayList<Double[]> result = new ArrayList<>();
				for (int i = 0; i < points.size(); i++)
				{
						Double[] p = points.get(i);
						result.add(new Double[]{p[0] - utm_centroid[0], p[1] - utm_centroid[1]});
				}
				return result;
		}

		private void lawnMower(ArrayList<Double[]> hull)
		{
				// TODO: fit rectangle over hull (rotated to have long axis sitting on hull diameter)
				// TODO:    a) find diameter of points
				int N = hull.size();
				int[] diameter_pair_indices = diameterPair(hull);
				ArrayList<Double[]> diameter_pair = new ArrayList<>();
				diameter_pair.add(hull.get(diameter_pair_indices[0]));
				diameter_pair.add(hull.get(diameter_pair_indices[1]));
				path_xy.addAll(diameter_pair); // diameter points are included automatically

				// TODO:    b) with line made from diameter pair, find point with most positive dot product
				// TODO:        Need the normal unit vector of the diameter line
				// TODO:    c) "", find point with most negative dot product
				Line diameter_line = new Line(0,
								hull.get(diameter_pair_indices[0]), hull.get(diameter_pair_indices[1]));
				ArrayList<Double[]> diameter_normal_ = unitNormalVector(diameter_pair);
				Double[] diameter_normal = diameter_normal_.get(0);

				double most_positive_dot = -1;
				double most_negative_dot = 1;
				Double[] most_positive = null;
				Double[] most_negative = null;
				Double[] diameter_midpoint = new Double[]{
								0.5*(diameter_line.p1[0] + diameter_line.p2[0]),
								0.5*(diameter_line.p1[1] + diameter_line.p2[1])};
				for (int i = 0; i < N; i++)
				{
						double dotn = dotNormalized(diameter_normal, difference(diameter_midpoint, hull.get(i)));
						if (dotn > most_positive_dot)
						{
								most_positive_dot = dotn;
								most_positive = hull.get(i);
						}
						else if (dotn < most_negative_dot)
						{
								most_negative_dot = dotn;
								most_negative = hull.get(i);
						}
				}

				double diameter_line_angle = Math.atan2(diameter_line.p2[1]-diameter_line.p1[1],
								diameter_line.p2[0]-diameter_line.p1[0]);
				double positive_line_angle = Math.atan2(most_positive[1]-diameter_line.p1[1],
								most_positive[0]-diameter_line.p1[0]);
				double negative_line_angle = Math.atan2(most_negative[1]-diameter_line.p1[1],
								most_negative[0]-diameter_line.p1[0]);

				double positive_distance = Math.abs(distance(diameter_line.p1, most_positive)
								*Math.sin(positive_line_angle - diameter_line_angle));
				double negative_distance = -1*Math.abs(distance(diameter_line.p1, most_negative)
								*Math.sin(negative_line_angle - diameter_line_angle));

				// TODO:    d) create lines from the hull, just like with spiral
				ArrayList<Line> hull_lines = new ArrayList<>();
				for (int i = 0; i < N; i++)
				{
						Double[] p1 = hull.get(i);
						int j = (i+1) % N; // wrap the index
						Double[] p2 = hull.get(j);
						hull_lines.add(new Line(i, p1, p2));
				}

				// TODO: calculate equations for lines back and forth across this rectangle
				// TODO:    a) start from the diameter
				// TODO:    b) while you haven't overshot yet, create a line from two points shifted by the positive transect distance
				// TODO:    c) find the left intersection, then the right (next iteration find right then left)
				// TODO:       NOTE the intersections are between the crossing line and the HULL lines!!!

				// TODO:    HOW DO YOU FIND INTERSECTIONS WITH THE CORRECT HULL LINE?
				// TODO:    Could create a more strict version of findIntersect that treats the lines as line segments,
				// TODO:        only returning a result if the intersection is located within both segments.
				// TODO:    In this manner, you would check for intersections with every hull line, and it should
				// TODO:        return exactly two points (unless it matches with a vertex exactly).
				// TODO:    Find the intersection closest to the most recent intersection, and add that first,
				// TODO:        then the other to create the zig zag.

				// TODO:    d) add these intersections onto the end of an array to create the back and forth motion
				// TODO:    e) continue until the line would be outside of the rectangle. Add the most positive dot product point as a final.
				// TODO:    f) next, starting at the diameter again, use the negative transect distance this and repeat the loop
				// TODO:       but instead of putting intersections at the end of the array, put them at zero
				// TODO:       OR flip the array and put them at the end
				// TODO:    g) and finally, once you overshoot, add the most negative dot product point as the final point

				// positive loop
				double shift_distance = transect_distance;
				do
				{
						// shift diameter
						Double[] dilated_normal_vector = new Double[]{
										shift_distance*diameter_normal[0],
										shift_distance*diameter_normal[1]};

						Line shifted_diameter = new Line(0,
										new Double[]{diameter_line.p1[0] + dilated_normal_vector[0], diameter_line.p1[1] + dilated_normal_vector[1]},
										new Double[]{diameter_line.p2[0] + dilated_normal_vector[0], diameter_line.p2[1] + dilated_normal_vector[1]});

						ArrayList<Double[]> path = new ArrayList<>();

						for (Line hull_line : hull_lines)
						{
								Intersection intersection = null;
								intersection = shifted_diameter.findIntersectionSegments(hull_line);
								if (intersection != null) path.add(intersection.p);
						}

						// find the point closer to the last point in path_xy
						// NOTE: assumes that you only exactly two intersections!
						if (distance(path.get(0), path_xy.get(path_xy.size()-1)) < distance(path.get(1), path_xy.get(path_xy.size()-1)))
						{
								path_xy.add(path.get(0));
								path_xy.add(path.get(1));
						}
						else
						{
								path_xy.add(path.get(1));
								path_xy.add(path.get(0));
						}

						shift_distance += transect_distance;
				} while (shift_distance < positive_distance);
				path_xy.add(most_positive);

				// negative loop
		}




}
