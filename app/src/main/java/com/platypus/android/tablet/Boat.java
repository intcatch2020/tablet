package com.platypus.android.tablet;

import android.os.Handler;
import android.os.Looper;

import com.mapbox.mapboxsdk.geometry.LatLng;
import com.platypus.crw.data.Pose3D;
import com.platypus.crw.data.Quaternion;
import com.platypus.crw.data.SensorData;
import com.platypus.crw.data.Utm;
import com.platypus.crw.data.UtmPose;

import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import java.net.InetSocketAddress;
import java.util.HashMap;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import javax.measure.unit.SI;

/**
 * Abstract class used by TeleOpPanel to interface with a boat.
 * <p>
 *     The abstract nature of this class allows for the GUI to function
 *     the same way with both real and simulated boats. Also, as long as
 *     the wireless communication provides a success/failure feedback mechanism,
 *     a child class could represent very different implementations
 *     of real boats.
 * </p>
 * <p>
 *     A common technique is for TeleOpPanel to provide Runnables that
 *     will update the GUI with the success or failure of a method call.
 *     For example, if the user tries to send a command to a real boat
 *     but the wireless communication fails, the "failure" runnable
 *     will cause a message to appear, informing the user of the failure.
 * </p>
 */
public abstract class Boat
{
		/**
		 * A string containin the ip address of the boat.
		 */
		private String ipAddressString;

		/**
		 * The name of the boat.
		 */
		String name;

		/**
		 * epoch time of the last successful communication sent to a boat
		 */
		AtomicLong time_of_last_connection = new AtomicLong(0);

		/**
		 * epoch time of the last joystick command sent to a boat
		 */
		AtomicLong time_of_last_joystick = new AtomicLong(0);

		/**
		 * True if the tablet has recently received messages from the Boat
		 */
		AtomicBoolean connected = new AtomicBoolean(false);

		/**
		 * True if the boat is currently autonomously navigating toward a waypoint
		 */
		AtomicBoolean autonomous = new AtomicBoolean(false);

		/**
		 * True if the boat has sent at least one sensor datum
		 */
		AtomicBoolean sensors_ready = new AtomicBoolean(false);

		/**
		 * The index of the waypoint that the boat is currently navigating towards.
		 * <p>
		 *     Used to update the colors of the lines displayed on the map.
		 * </p>
		 */
		AtomicInteger current_waypoint_index = new AtomicInteger(-1);

		/**
		 * Android logs generated in this class will be tagged with this.
		 */
		String logTag = "Boat";

		/**
		 * The current location of the boat.
		 */
		private LatLng current_location = null;

		/**
		 * The home location of the boat.
		 */
		LatLng home_location = null;

		/**
		 * Object used for threadsafe location updates
		 */
		private final Object location_lock = new Object();

		LatLng new_crumb_LatLng = null;
		UTM new_crumb_UTM = null;

		/**
		 * HashMap from a crumb's unique ID to the latitude/longitude of that crumb
		 */
		HashMap<Long, double[]> crumb_map= new HashMap<>();

		/**
		 * Object used for threadsafe crumb updates
		 */
		final Object crumb_lock = new Object();

		/**
		 * The yaw of the boat, wrapped to the [-pi, pi] interval
		 */
		private double current_yaw = 0.0;

		/**
		 * Object used for threadsafe yaw updates
		 */
		private final Object yaw_lock = new Object();
		double[][] PID_gains = {{0., 0., 0.}, {0., 0., 0.}}; // thrust, heading

		/**
		 * Object used for threadsafe PID updates
		 */
		final Object PID_lock = new Object();

		/**
		 * "axis" parameter used by Platypus Core "setGains" call to update thrust PID
		 */
		final int THRUST_GAIN_AXIS = 0;

		/**
		 * "axis" parameter used by Platypus Core "setGains" call to update heading PID
		 */
		final int RUDDER_GAIN_AXIS = 5;

		/**
		 * "axis" parameter used by Platypus Core "setGains" call to control the sampler
		 */
		final int SAMPLER_GAIN_AXIS = 7;

		/**
		 * Handles the execution of any polling thread Runnables
		 */
		ScheduledThreadPoolExecutor polling_thread_pool;

		/**
		 * Used to post Runnables that update the GUI
		 */
		Handler uiHandler = new Handler(Looper.getMainLooper());

		/**
		 * Holds the most recent sensor datum sent by the boat
		 */
		SensorData lastSensorDataReceived;

		/**
		 * Object used for threadsafe sensor datum updates
		 */
		final Object sensor_lock = new Object();

		/**
		 * Current waypoint status of the boat
		 */
		String waypoint_state;
		final Object waypoint_state_lock = new Object();
		boolean[] sampler_running = {false, false, false, false};

		/**
		 * The color of the boat icon
		 */
		private int boat_color;

		/**
		 * The color of the boat's assigned path
		 */
		private int line_color;

		/**
		 * Abstract method that allows a Boat child class to assign Runnables to be executed as callbacks to pose, sensor, waypoint status, and crumb events.
		 * @param poseListenerCallback  Runnable that is typically used when a new pose is received
		 * @param sensorListenerCallback  Runnable that is typically used when a new sensor datum is received
		 * @param waypointListenerCallback  Runnable that is typically used when a new waypoint status is received
		 * @param crumbListenerCallback  Runnable that is typically used when a new crumb is received
		 */
		abstract public void createListeners(
						final Runnable poseListenerCallback,
						final Runnable sensorListenerCallback,
						final Runnable waypointListenerCallback,
						final Runnable crumbListenerCallback);

		/**
		 * Abstract method to provide a list of waypoints to a Boat to navigate towards
		 * @param waypoints  nested array of doubles represents a sequence of latitude/longitude pairs
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void startWaypoints(final double[][] waypoints, final Runnable failureCallback);

		/**
		 * Abstract method to stop any currently running autonomous navigation
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void stopWaypoints(final Runnable failureCallback);

		/**
		 * Abstract method to send joystick information to the boat
		 * @param thrust  thrust signal from the joystick
		 * @param heading  heading/rudder signal from the joystick
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void updateControlSignals(final double thrust, final double heading, final Runnable failureCallback);

		/**
		 * Abstract method to set the value of the boat's "autonomous" boolean
		 * @param b  True if the boat should be autonomous, false otherwise
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void setAutonomous(final boolean b, final Runnable failureCallback);

		/**
		 * Abstract method to set the PID control parameters of the boat.
		 * @param thrustPID  double array with 3 values representing P, I, and D for thrust control
		 * @param headingPID  double array with 3 values representing P, I, and D for heading/rudder control
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void setPID(final double[] thrustPID, final double[] headingPID, final Runnable failureCallback);

		/**
		 * Abstract method to send a single waypoint for the boat to navigate towards
		 * @param waypoint  latitude/longtidue of the waypoint
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void addWaypoint(double[] waypoint, final Runnable failureCallback);

		/**
		 * Abstract method to send an autonomous behavior definition to the boat
		 * @param apm  an AutonomousPredicateMessage's JSON string definition
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void sendAutonomousPredicateMessage(String apm, final Runnable failureCallback);
		abstract public void setAddress(InetSocketAddress a);
		abstract public InetSocketAddress getIpAddress();

		/**
		 * Abstract method to send a sampler control command to the boat, starting a jar
 		 * @param jar_number  the number of the jar to start
		 * @param TimerStartRunnable  Runnable typically used to display a jar countdown timer on the GUI
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void startSample(final int jar_number, final Runnable TimerStartRunnable, final Runnable failureCallback);

		/**
		 * Abstract method to send a sampler control command to the boat, stopping a jar
		 * @param jar_number  the number of the jar to stop
		 * @param successCallback  Runnable that is typically used when wireless communication succeeds
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void stopSample(final int jar_number, final Runnable successCallback, final Runnable failureCallback);

		/**
		 * Abstract method to send a sampler control command to the boat, stopping all jars
		 * @param successCallback  Runnable that is typically used when wireless communication succeeds
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void stopSampleAll(final Runnable successCallback, final Runnable failureCallback);

		/**
		 * Abstract method to send a sampler control command to the boat, resetting the sampler
		 * @param successCallback  Runnable that is typically used when wireless communication succeeds
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void resetSampler(final Runnable successCallback, final Runnable failureCallback);

		/**
		 * Abstract method to set the home location of the boat
		 * @param home  the new home location
		 * @param successCallback  Runnable that is typically used when wireless communication succeeds
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void setHome(final LatLng home, final Runnable successCallback, final Runnable failureCallback);

		/**
		 * Abstract method to command the boat to go home
		 * @param failureCallback  Runnable that is typically used when wireless communication fails
		 */
		abstract public void goHome(final Runnable failureCallback);

		public String getName() { return name; }
		void setBoatColor(int _color) { boat_color = _color; }
		int getBoatColor() { return boat_color; }
		void setLineColor(int _color) { line_color = _color; }
		int getLineColor() { return line_color; }
		void setIpAddressString(String addr)
		{
				ipAddressString = addr;
		}
		String getIpAddressString()
		{
				return ipAddressString;
		}

		LatLng getHome()
		{
				return home_location;
		}

		double getYaw()
		{
				synchronized (yaw_lock)
				{
						return current_yaw;
				}
		}

		void setYaw(double yaw)
		{
				while (Math.abs(yaw) > Math.PI)
				{
						yaw -= 2*Math.PI*Math.signum(yaw);
				}
				synchronized (yaw_lock)
				{
						current_yaw = yaw;
				}
		}

		SensorData getLastSensorDataReceived()
		{
				synchronized (sensor_lock)
				{
						return lastSensorDataReceived;
				}
		}

		String getWaypointState()
		{
				synchronized (waypoint_state_lock)
				{
						return waypoint_state;
				}
		}

		void setConnected(boolean b)
		{
				connected.set(b);
				time_of_last_connection.set(System.currentTimeMillis());
		}

		boolean isConnected()
		{
				return connected.get();
		}

		LatLng getLocation()
		{
				synchronized (location_lock)
				{
						return current_location;
				}
		}

		void setLocation(LatLng loc)
		{
				synchronized (location_lock)
				{
						current_location = loc;
				}
		}

		int getWaypointsIndex()
		{
				return current_waypoint_index.get();
		}

		public double[][] getPID()
		{
				synchronized (PID_lock)
				{
						return PID_gains.clone();
				}
		}

		LatLng getNewCrumb()
		{
				synchronized (crumb_lock)
				{
						return new_crumb_LatLng;
				}
		}

		/**
		 * Convert from jscience LatLng object to MapBox LatLng object
		 */
		static com.mapbox.mapboxsdk.geometry.LatLng jscienceLatLng_to_mapboxLatLng(org.jscience.geography.coordinates.LatLong jlatlng)
		{
				return new LatLng(
								jlatlng.latitudeValue(SI.RADIAN)*180./Math.PI,
								jlatlng.longitudeValue(SI.RADIAN)*180./Math.PI);
		}

		/**
		 * Convert from UtmPose location to a UTM location
		 */
		private static UTM UtmPose_to_UTM(UtmPose utmPose)
		{
				return UTM.valueOf (
								utmPose.origin.zone,
								utmPose.origin.isNorth ? 'T' : 'L',
								utmPose.pose.getX(),
								utmPose.pose.getY(),
								SI.METER
				);
		}

		/**
		 * Ensure an angle is within the the interval [-pi, pi]
		 */
		static double normalizeAngle(double angle) {
				while (angle > Math.PI)
						angle -= 2 * Math.PI;
				while (angle < -Math.PI)
						angle += 2 * Math.PI;
				return angle;
		}
}