package com.platypus.android.tablet.Path;

import android.util.Log;
import android.util.Pair;

import com.mapbox.mapboxsdk.geometry.LatLng;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

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
				/*
				boolean eq(Intersection j)
				{
						return ((first == j.first && second == j.second) || (first == j.second && second == j.first));
				}
				*/
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
				boolean eq(Line j)
				{
						return index == j.index;
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
		}

		/*
		class IntersectionTriplet
		{
				Pair<Integer, Integer> i;
				Pair<Integer, Integer> j;
				Pair<Integer, Integer> skip;
				IntersectionTriplet(Pair<Integer, Integer> i_, Pair<Integer, Integer> j_, Pair<Integer, Integer> skip_)
				{
						i = i_;
						j = j_;
						skip = skip_;
				}
		}
		*/

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
						Log.d(logTag, String.format("final sequence point = [%.1f, %.1f]", p[0], p[1]));
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

		/*
		private double minDistanceToCentroid(ArrayList<Double[]> points)
		{
				double min_distance = -1;
				for (int i = 0; i < points.size(); i++)
				{
						double d = distanceToCentroid(points.get(i));
						if (min_distance < 0 || d < min_distance) min_distance = d;
				}
				return min_distance;
		}

		private double maxDistanceToCentroid(ArrayList<Double[]> points)
		{
				double max_distance = -1;
				for (int i = 0; i < points.size(); i++)
				{
						double d = distanceToCentroid(points.get(i));
						if (max_distance < 0 || d > max_distance) max_distance = d;
				}
				return max_distance;
		}
		*/

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

		/*
		private boolean isMergeValid(ArrayList<Double[]> hull, Double[] i12, Double[] i23, Double[] i13)
		{
				double dist_12 = distanceToCentroid(i12);
				double dist_23 = distanceToCentroid(i23);
				double dist_13 = distanceToCentroid(i13);
				Log.i(logTag, String.format("dist_12 = %.1f,  dist_23 = %.1f,  dist_13 = %.1f", dist_12, dist_23, dist_13));

				if (dist_13 < dist_12 && dist_13 < dist_23 && hull.size() > 3 && isInsideHull(hull, i13))
				{

						//How to judge if a point is above or below a line:
						//equation for line --> y = mx + b
						//1) using centroid and i12, find m and b
						//	a) know change in y, now change in x --> m
						//	b) b = y - mx --> plug in m, x and y of centroid to find b
						//2) plug in i23[0] i23[1] for x and y --> mx + b - y = positive or negative?
						//3) plug in i13[0] i13[1] for x and y --> mx + b - y = positive or negative?
						//4) If the signs in #2 and #3 are the same, then i13 can be valid

						Double[] diff = difference(local_centroid, i12);
						double m = diff[1]/diff[0];
						double b = i12[1] - m*i12[0];
						if (Math.signum((m*i23[0] + b - i23[1])*(m*i13[0] + b - i13[1])) > 0)
						{
								// same sign - valid
								return true;
						}
				}
				return false;
		}
		*/

		private Boolean isInsideHull(ArrayList<Double[]> hull, Double[] p)
		{
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

		/*
		private ArrayList<Double[]> mergeClosePoints(ArrayList<Double[]> points, double merging_distance)
		{
				// merge one pair at a time, recurse until no pairs can be merged OR only 3 points remain OR all points would merge
				if (points.size() < 4) return points;
				ArrayList<Double[]> result = new ArrayList<>();
				double[][] pairwise_distances = pairwiseDistances(points);
				HashMap<Integer, Integer> merging_map = new HashMap<>();
				for (int i = 0; i < points.size(); i++)
				{
						for (int j = 0; j < points.size(); j++)
						{
								if ((i != j) && (pairwise_distances[i][j] < merging_distance))
								{
										if (!merging_map.containsKey(j)) merging_map.put(i, j);
										break;
								}
						}
				}
				if (merging_map.size() == 0) return points;
				Log.w(logTag, String.format("There are %d points to merge", merging_map.size()));

				// extract only the first merge pair
				int a = -1; // indices
				int b = -1;
				for (Map.Entry<Integer, Integer> entry : merging_map.entrySet())
				{
						a = entry.getKey();
						b = entry.getValue();
						Log.d(logTag, String.format("Merging index %d and %d", a, b));
						break;
				}

				for (int i = 0; i < points.size(); i++)
				{
						if (i == a)
						{
								// merge these two
								Double[] pa = points.get(a);
								Double[] pb = points.get(b);
								result.add(new Double[]{0.5*pa[0]+0.5*pb[0], 0.5*pa[1]+0.5*pb[1]});
						}
						else if (i != b)
						{
								result.add(points.get(i));
						}
				}
				return mergeClosePoints(result, merging_distance);
		}
		*/

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

		/*
		private ArrayList<Double[]> lineSegmentDifferences(ArrayList<Double[]> points)
		{
				ArrayList<Double[]> rolled_points = shiftArrayList(points, 1);
				ArrayList<Double[]> result = new ArrayList<>();
				for (int i = 0; i < points.size(); i++)
				{
						Double[] a = points.get(i);
						Double[] b = rolled_points.get(i);
						result.add(new Double[]{b[0]-a[0], b[1]-a[1]});
				}
				return result;
		}
		*/
		private ArrayList<Double> lineSegmentNormalAngles(ArrayList<Double[]> points)
		{
				ArrayList<Double[]> rolled_points = shiftArrayList(points, 1);
				ArrayList<Double> result = new ArrayList<>();
				for (int i = 0; i < points.size(); i++)
				{
						Double[] a = rolled_points.get(i);
						Double[] b = points.get(i);
						//Log.v(logTag, String.format("a = [%.0f, %.0f], b = [%.0f, %.0f]", a[0], a[1], b[0], b[1]));
						double raw_angle = Math.atan2(b[1]-a[1], b[0]-a[0]) + Math.PI/2.0;
						result.add(wrapToPi(raw_angle));
				}
				return result;
		}

		/*
		private Double[] findIntersection(Double[] line1_point1, Double[] line1_point2, Double[] line2_point1, Double[] line2_point2)
		{
				Double[] d1 = difference(line1_point1, line1_point2);
				Double[] d2 = difference(line2_point1, line2_point2);
				Double[] dp = difference(line2_point1, line1_point1);
				Double[] d1p = new Double[]{-d1[1], d1[0]};
				double denominator = dot(d1p, d2);
				double numerator = dot(d1p, dp);
				double c = numerator/denominator;
				return new Double[]{c*d2[0] + line2_point1[0], c*d2[1] + line2_point1[1]};
		}
		*/

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
				//ArrayList<Double> rolled_angles = shiftArrayList(normal_angles, -1);

				// find the points if they were moved inward along their normal by the transect distance
				//ArrayList<Double[]> new_segments_point_1 = new ArrayList<>();
				//ArrayList<Double[]> new_segments_point_2 = new ArrayList<>();
				int N = previous_hull.size();
				for (int i = 0; i < N; i++)
				{
						Double[] p1 = previous_hull.get(i);
						int j = i+1 % N; // wrap the index
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

				// find all possible intersections
				//Set<Intersection> intersections = new HashSet<>();
				HashMap<Pair<Integer, Integer>, Intersection> intersections = new HashMap<>();
				/*
				for (Line li : lines)
				{
						for (Line lj : lines)
						{
								if (!li.eq(lj))
								{
										if (!intersections.containsKey(new Pair<>(li.index, lj.index)))
										{
												Intersection new_intersection = li.findIntersection(lj);
												// include both i,j and j,i keys
												intersections.put(new Pair<>(li.index, lj.index), new_intersection);
												intersections.put(new Pair<>(lj.index, li.index), new_intersection);
										}
								}
						}
				}
				*/

				// find all intersections
				int skip_level = 1;
				int i = 0;
				do
				{
						int j = (i + skip_level) % N;
						Pair<Integer, Integer> pair = new Pair<>(i, j);
						Pair<Integer, Integer> reverse_pair = new Pair<>(i, j);
						if(intersections.containsKey(pair) || intersections.containsKey(reverse_pair))
						{
								break;
						}

						Intersection ij = lines.get(i).findIntersection(lines.get(j));
						if (isInsideHull(previous_hull, ij.p))
						{
								intersections.put(pair, ij);
								// check for intersection triplets
								if (skip_level > 1)
								{
										////////////////////////////////////
										// TODO: need to find the two points we might be replacing
										// TODO:    we know that the indices are in between i and j, but in the wrapping index space, not arithmetically
										// TODO:    we always travel toward increasing (but wrapped) index, i.e. 3, 4, 5, 1, 2, etc.
										//
										Pair<Integer, Integer> parent_1 = new Pair<>(i, (i + skip_level-1) % N);
										Pair<Integer, Integer> parent_2 = new Pair<>((i + 1) % N, j);
										////////////////////////////////////

										// check if i,j is closer to the centroid than the parents
										if
										(
												intersections.get(new Pair<>(i, j)).distance_to_center < intersections.get(parent_1).distance_to_center
												&&
												intersections.get(new Pair<>(i, j)).distance_to_center < intersections.get(parent_2).distance_to_center
										)
										{
										// lines will merge, so only retain i,j
										intersections.get(new Pair<>(i, j)).usable = true;
										intersections.get(parent_1).usable = false;
										intersections.get(parent_2).usable = false;
										}
										else
										{
												// i,j was not closer, do not use it
												intersections.get(new Pair<>(i, j)).usable = false;
										}
								} // if skip_level > 1
						} // if i,j inside hull

						i += 1;
						if (i == N)
						{
								i = 0;
								skip_level += 1;
						}
				} while (true);

				ArrayList<Intersection> new_hull = new ArrayList<>();
				for (Intersection intersection : intersections.values())
				{
						if (intersection.usable) new_hull.add(intersection);
				}

				// if there aren't at least three accepted vertices truncate
				if (new_hull.size() < 3)
				{
						Log.i(logTag, "There are less than 3 accepted vertices, truncating");
						path_xy.add(local_centroid);
						return;
				}



				// shift new_segments_point_2 so that the new lines have their indices aligned properly
				//new_segments_point_2 = shiftArrayList(new_segments_point_2, 1);

				/*
				// find intersections between the new lines
				ArrayList<Double[]> new_segments_point_1_rolled = shiftArrayList(new_segments_point_1, 1);
				ArrayList<Double[]> new_segments_point_2_rolled = shiftArrayList(new_segments_point_2, 1);
				ArrayList<Double[]> new_segments_point_1_twice_rolled = shiftArrayList(new_segments_point_1, 2);
				ArrayList<Double[]> new_segments_point_2_twice_rolled = shiftArrayList(new_segments_point_2, 2);
				ArrayList<Double[]> accepted = new ArrayList<>();
				Set<List<Double>> intersections = new HashSet<>(); // cannot use Double[] (an array) in a set!
				Set<List<Double>> rejected = new HashSet<>();
				int mergedCount = 0;
				for (int segi = 0; segi < new_segments_point_1.size(); segi++)
				{
						Double[] p1 = previous_hull.get(segi);
						Log.v(logTag, String.format("base vertex = [%.1f, %.1f]", p1[0], p1[1]));
						Log.v(logTag, String.format("vertices, line 1 = [%.1f,%.1f], [%.1f, %.1f]",
										new_segments_point_1.get(segi)[0],
										new_segments_point_1.get(segi)[1],
										new_segments_point_2.get(segi)[0],
										new_segments_point_2.get(segi)[1]));
						Log.v(logTag, String.format("vertices, line 2 = [%.1f,%.1f], [%.1f, %.1f]",
										new_segments_point_1_rolled.get(segi)[0],
										new_segments_point_1_rolled.get(segi)[1],
										new_segments_point_2_rolled.get(segi)[0],
										new_segments_point_2_rolled.get(segi)[1]));
						Log.v(logTag, String.format("vertices, line 3 = [%.1f,%.1f], [%.1f, %.1f]",
										new_segments_point_1_twice_rolled.get(segi)[0],
										new_segments_point_1_twice_rolled.get(segi)[1],
										new_segments_point_2_twice_rolled.get(segi)[0],
										new_segments_point_2_twice_rolled.get(segi)[1]));

						Double[] intersection_12 = findIntersection(
										new_segments_point_1.get(segi),
										new_segments_point_2.get(segi),
										new_segments_point_1_rolled.get(segi),
										new_segments_point_2_rolled.get(segi));

						Double[] intersection_23 = findIntersection(
										new_segments_point_1_rolled.get(segi),
										new_segments_point_2_rolled.get(segi),
										new_segments_point_1_twice_rolled.get(segi),
										new_segments_point_2_twice_rolled.get(segi));

						Double[] intersection_13 = findIntersection(
										new_segments_point_1.get(segi),
										new_segments_point_2.get(segi),
										new_segments_point_1_twice_rolled.get(segi),
										new_segments_point_2_twice_rolled.get(segi));

						Log.v(logTag, String.format("i12: [%.1f, %.1f]", intersection_12[0], intersection_12[1]));
						Log.v(logTag, String.format("i23: [%.1f, %.1f]", intersection_23[0], intersection_23[1]));
						Log.v(logTag, String.format("i13: [%.1f, %.1f]", intersection_13[0], intersection_13[1]));

						double dist_12 = distanceToCentroid(intersection_12);
						double dist_23 = distanceToCentroid(intersection_23);
						double dist_13 = distanceToCentroid(intersection_13);
						Log.v(logTag, String.format("dist_12 = %.1f,  dist_23 = %.1f,  dist_13 = %.1f", dist_12, dist_23, dist_13));
						boolean validMerge = isMergeValid(previous_hull, intersection_12, intersection_23, intersection_13);
						if (validMerge)
						{
								// need to merge - only retain intersection_13 and skip ahead
								// NOTE: this idea totally breaks down if we already have a triangle
								Log.v(logTag, String.format("segi=%d: merging two intersections", segi));
								intersections.add(Arrays.asList(intersection_13[0], intersection_13[1]));
								rejected.add(Arrays.asList(intersection_12[0], intersection_12[1]));
								rejected.add(Arrays.asList(intersection_23[0], intersection_23[1]));
								mergedCount += 1;
						}
						else
						{
								// no need to merge - only retain intersection_12 and move on normally
								Log.v(logTag, String.format("segi=%d: normal intersection", segi));
								intersections.add(Arrays.asList(intersection_12[0], intersection_12[1]));
								intersections.add(Arrays.asList(intersection_23[0], intersection_23[1]));
						}
				}

				for (List<Double> intersection : intersections)
				{
						if (!rejected.contains(intersection))
						{
								accepted.add(new Double[]{intersection.get(0), intersection.get(1)});
						}
				}

				// if there aren't at least three accepted vertices truncate
				if (accepted.size() < 3)
				{
						Log.i(logTag, "There are less than 3 accepted vertices, truncating");
						path_xy.add(local_centroid);
						return;
				}

				Log.d(logTag, String.format("Accepted %d intersection vertices", accepted.size()));

				// the accepted points will be out of order. Get their convex hull for the order
				ArrayList<Integer> convex_indices = GrahamScan.getConvexHull(accepted);
				ArrayList<Double[]> reordered = new ArrayList<>();
				for (Integer i : convex_indices)
				{
						reordered.add(accepted.get(i));
						Log.v(logTag, String.format("New vertex [%.1f, %.1f]", accepted.get(i)[0], accepted.get(i)[1]));
				}

				// Check for overshoot #1: Always check if a point is inside the hull
				ArrayList<Boolean> is_inside = isInsideHull(previous_hull, reordered);
				for (int i = 0; i < reordered.size(); i++)
				{
						if (!is_inside.get(i))
						{
								Log.i(logTag, String.format("Point %d is outside of the previous hull, truncating", i));
								path_xy.add(local_centroid);
								return;
						}
				}
				*/

				/*
				// Check for overshoot #2: max distance to centroid must always decrease
				if (max_dist_to_centroid < 0)
				{
						// the first time
						max_dist_to_centroid = maxDistanceToCentroid(previous_hull);
				}
				double new_max_dist = maxDistanceToCentroid(reordered);
				if (new_max_dist < max_dist_to_centroid)
				{
						max_dist_to_centroid = new_max_dist; // continue normally
				}
				else
				{
						Log.i(logTag, "Max distance to centroid did not decrease, truncating");
						path_xy.add(local_centroid);
						return;
				}
				*/

				/*
				// to create a *closed-hull* spiral we need to shift back by one again (3rd must become 1st)
				// TODO: Find a "primary vertex" in the original first hull that we will use as a tool to order all subsequent hulls
				// TODO: To find the shift necessary to create that nice inward spiral,
				// TODO:      we start by finding the index of the point closest to the primary vertex
				// TODO: Once we know that, shift by the difference between this and the inward_count (the number of hulls inward)
				// TODO: If done correctly, we can figure out how to perfectly align the order of points
				int shift = -1*inward_count;
				shift += mergedCount;
				reordered = shiftArrayList(reordered, shift);

				local_centroid = calculateCentroid(reordered);
				Log.i(logTag, String.format("New local centroid = [%.1f, %.1f]", local_centroid[0], local_centroid[1]));

				path_xy.addAll(reordered);
				path_xy.add(reordered.get(0).clone()); // close the hull before moving to inward hull

				inward_count += 1;
				inwardNextHull(reordered, inward_count);
				*/
		}

		private void convexHull() throws Exception
		{
				// set convex_xy based on original_points
				ArrayList<Double[]> utm_points = latLngToUTM(original_points);
				ArrayList<Integer> convex_indices = GrahamScan.getConvexHull(utm_points);
				ArrayList<Double[]> utm_hull = new ArrayList<>();
				for (Integer i : convex_indices)
				{
						utm_hull.add(utm_points.get(i));
				}
				convex_xy = zeroCentroid(utm_hull);
				// Double[] should_be_zero_centroid = calculateCentroid(convex_xy);
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
