package com.platypus.android.tablet.Path;

import android.util.Log;
import android.util.Pair;

import com.mapbox.mapboxsdk.geometry.LatLng;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

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
								// TODO: fit rectangle over hull (rotated to have long axis sitting on hull diameter)
								// TODO: calculate equations for lines back and forth across this rectangle
								// TODO: calculate intersections between hull and the lawnmower lines
								// TODO: properly order the intersections to generate the lawnmower path
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
				return a[0]*b[0] + a[1]*b[1];
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
						path_xy.add(path_xy.get(0).clone()); // close the hull
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

				// TODO: find point closest to first (and last) point in previous_hull
				// TODO: shift new hull until you are one off from that
				// TODO: this creates the spiral pattern we want


				path_xy.addAll(new_hull);
				path_xy.add(new_hull.get(0).clone()); // close the hull before moving to inward hull

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




}
