package com.platypus.android.tablet;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.measure.unit.NonSI;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import com.mapbox.mapboxsdk.Mapbox;
import com.mapbox.mapboxsdk.annotations.MarkerView;
import com.mapbox.mapboxsdk.annotations.MarkerViewOptions;
import com.mapbox.mapboxsdk.annotations.Polyline;
import com.mapbox.mapboxsdk.annotations.PolylineOptions;
import com.mapbox.mapboxsdk.camera.CameraPosition;
import com.mapbox.mapboxsdk.camera.CameraUpdateFactory;
import com.mapbox.mapboxsdk.constants.Style;
import com.mapbox.mapboxsdk.geometry.LatLng;
import com.mapbox.mapboxsdk.annotations.Marker;
import com.mapbox.mapboxsdk.annotations.MarkerOptions;
import com.mapbox.mapboxsdk.annotations.Icon;
import com.mapbox.mapboxsdk.annotations.IconFactory;
import com.mapbox.mapboxsdk.maps.MapView;
import com.mapbox.mapboxsdk.maps.MapboxMap;
import com.mapbox.mapboxsdk.maps.OnMapReadyCallback;

import android.app.AlertDialog;
import android.app.NotificationManager;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.PorterDuff;
import android.graphics.drawable.Drawable;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.media.Ringtone;
import android.media.RingtoneManager;
import android.net.ConnectivityManager;

import android.net.Uri;
import android.os.CountDownTimer;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;

import android.preference.PreferenceManager;
import android.support.annotation.LayoutRes;
import android.support.annotation.NonNull;
import android.support.v4.app.NotificationCompat;
import android.support.v4.content.ContextCompat;
import android.view.MenuItem;
import android.view.View;

import com.platypus.android.tablet.Path.AreaType;
import com.platypus.android.tablet.Path.Path;
import com.platypus.android.tablet.Path.Region;
import com.platypus.crw.CrwNetworkUtils;
import com.platypus.crw.VehicleServer;
import com.platypus.crw.data.SensorData;

import android.app.Activity;
import android.content.Context;

import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.PopupMenu;
import android.widget.RadioButton;
import android.widget.RelativeLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import android.app.Dialog;

import android.view.View.OnClickListener;

import com.platypus.android.tablet.Joystick.*;

public class TeleOpPanel extends Activity implements SensorEventListener
{
		HashMap<String, Boat> boats_map = new HashMap<>();
		HashMap<String, MarkerViewOptions> boat_markers_map = new HashMap<>();
		HashMap<String, MarkerViewOptions> home_markers_map = new HashMap<>();
		HashMap<String, Path> path_map = new HashMap<>();
		HashMap<String, ArrayList<Polyline>> waypath_outline_map = new HashMap<>();
		HashMap<String, ArrayList<Polyline>> waypath_top_map = new HashMap<>();
		//HashMap<String, Polyline> boat_to_wp_line_map = new HashMap<>();
		HashMap<String, Integer> current_wp_index_map = new HashMap<>();
		HashMap<String, Integer> old_wp_index_map = new HashMap<>();
		HashMap<String, ArrayList<Marker>> crumb_markers_map = new HashMap<>();

		// TODO: key: boat name, value: {key: sensor's (channel, type) hash, value: Mapbox marker objects}
		HashMap<String, HashMap<Integer, ArrayList<Marker>>> sensordata_markers_map = new HashMap<>();

		HashMap<String, PlatypusMarkerTypes> marker_types_map = new HashMap<>();

		// TODO: if I'm using a recycler view for the APM GUI, that has a List of views
		// TODO: Doesn't this key: id, value: APM hashmap have to align with that list?
		HashMap<Integer, AutonomousPredicateMessage> ap_messages_map = new HashMap<>();

		void newAutonomousPredicateMessage(String name, String action,
		                                   String trigger, long interval, boolean ends)
		{
				AutonomousPredicateMessage apm = new AutonomousPredicateMessage(name, action, trigger, interval, ends);
				if (apm.generateStringifiedJSON() != null)
				{
						ap_messages_map.put(ap_messages_map.size(), apm);
				}
		}

		void sendAllAPMessages()
		{
				for (AutonomousPredicateMessage apm : ap_messages_map.values())
				{
						if (!apm.getAcknowledged())
						{
								Boat boat = currentBoat();
								// TODO: core lib call with void listener, if completed, set acknowledged to true
								boat.sendAutonomousPredicateMessage(apm.generateStringifiedJSON(), null);
						}
				}
		}

		final Context context = this;
		TextView ipAddressBox = null;
		RelativeLayout linlay = null;

		Button connect_button = null;
		RadioButton real_boat_button = null;
		RadioButton sim_boat_button = null;
		boolean use_real_boat = true;
		Button advanced_options_button = null;
		Button center_on_boat_button = null;
		Button center_on_operator_button = null;
		Button start_wp_button = null;
		Button pause_wp_button = null;
		boolean paused = false;
		Button stop_wp_button = null;
		Button undo_last_wp_button = null;
		Button remove_all_wp_button = null;
		Button drop_wp_button = null;
		Button normal_path_button = null;
		Button spiral_button = null;
		Button lawnmower_button = null;
		Button reverse_order_button = null;
		Button simple_path_button = null;
		Button jar1_button = null;
		Button jar2_button = null;
		Button jar3_button = null;
		Button jar4_button = null;
		Button sampler_reset_button = null;
		Button sampler_stop_all_button = null;

		TextView jar1_text = null;
		TextView jar2_text = null;
		TextView jar3_text = null;
		TextView jar4_text = null;

		SensorStuff sensor_stuff = null;
		SavedWaypointsStuff saved_waypoint_stuff = null;

		TextView battery_value = null;
		TextView waypointInfo = null;
		TextView path_length_value = null;
		TextView rc_override_warning = null;

		JoystickView joystick;
		private boolean speed_spinner_erroneous_call = true;
		Spinner speed_spinner = null;
		Spinner available_boats_spinner = null;
		ColorfulSpinnerAdapter available_boats_spinner_adapter = null;

		Handler uiHandler = new Handler(Looper.getMainLooper()); // anything post to this is run on the main GUI thread

		double currentTransectDist = 20;

		MapView mv;
		MapboxMap mMapboxMap;
		IconFactory mIconFactory;

		private File mLogFile;
		private PrintWriter mLogWriter;
		private long mLogStartTime;

		int current_wp_list_selected = -1; // which element selected
		LatLng pHollowStartingPoint = new LatLng((float) 40.436871, (float) -79.948825);
		LatLng initialPan = new LatLng(0, 0);
		boolean setInitialPan = true;

		SensorManager senSensorManager;
		Sensor senAccelerometer;

		public static double THRUST_MIN = -1.0;
		public static double THRUST_MAX = 0.3;
		public static double RUDDER_MIN = -1.0;
		public static double RUDDER_MAX = 1.0;

		public EditText ipAddressInput = null;
		public EditText transect_distance_input = null;

		public static String textIpAddress;
		public static String boatPort = "11411";

		double[] tPID = {.2, .0, .0};
		double[] rPID = {1, 0, .2};

		double battery_voltage = 0.0;

		ArrayList<LatLng> waypoint_list = new ArrayList<LatLng>(); // waypoints selected by the user
		ArrayList<Marker> marker_list = new ArrayList<>(); // markers associated with those waypoints
		ArrayList<Polyline> outline_list = new ArrayList<>(); // lines generated by user input but not yet assigned to any boat
		ArrayList<Polyline> topline_list = new ArrayList<>(); // lines generated by user input but not yet assigned to any boat
		Path unowned_path = new Path(); // a path generated by user input but not yet assigned to any boat
    
		final Object _wpGraphicsLock = new Object();

		private static final String logTag = "TeleOpPanel"; //TeleOpPanel.class.getName();

		NotificationManager notificationManager;
		Uri soundUri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION);
		long sleep_start_time = 0;
		boolean alarm_on = false;
		final Object _batteryVoltageLock = new Object();
		Ringtone alarm_ringtone;
		Uri alarmUri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_ALARM);

		LocationListener location_listener;
		final Object tablet_location_lock = new Object();
		Double[] tablet_latlng = new Double[] {0.0, 0.0};
		boolean tablet_gps_fix = false;
		MarkerViewOptions tablet_location_markerviewoptions = null;
		double distance_to_current_boat;
		AtomicBoolean current_boat_is_connected = new AtomicBoolean(false);

		int boat_color_count = 0;
		Map<Integer, Map<String, Integer>> color_map = new HashMap<>();

		public Icon colorIconFromDrawable(Drawable drawable, Integer color)
		{
				if (color != null)
				{
						PorterDuff.Mode mMode = PorterDuff.Mode.MULTIPLY;
						drawable.setColorFilter(color, mMode);
				}
				// https://github.com/mapbox/mapbox-gl-native/issues/8185
				Bitmap bitmap = Bitmap.createBitmap(drawable.getIntrinsicWidth(), drawable.getIntrinsicHeight(), Bitmap.Config.ARGB_8888);
				Canvas canvas = new Canvas(bitmap);
				drawable.setBounds(0, 0, canvas.getWidth(), canvas.getHeight());
				drawable.draw(canvas);
				return mIconFactory.fromBitmap(bitmap);
		}

		class ColorfulSpinnerAdapter extends ArrayAdapter<String>
		{
				public ColorfulSpinnerAdapter(@NonNull Context context, @LayoutRes int resource)
				{
						super(context, resource);
				}
				@Override
				public View getDropDownView(int position, View convertView, ViewGroup parent)
				{
						View view = super.getDropDownView(position, convertView, parent);
						applyColor(view, position);
						return view;
				}

				private void applyColor(View view, int position)
				{
						while (position > 5) // need to be within the limited set of 6 colors
						{
								position -= 6;
						}
						view.setBackgroundColor(color_map.get(position).get("boat"));
				}
				// TODO: don't rely on a hardcoded color circuit
		}

		void startNewBoat(final String boat_name)
		{
				// generate Boat object and put it into the boat_map
				Boat newBoat;
				if (use_real_boat)
				{
						newBoat = new RealBoat(boat_name);
				}
				else
				{
						// set the initial UTM to be the center of the current map view
						LatLng center = mMapboxMap.getCameraPosition().target;
						UTM initial_simulated_utm = UTM.latLongToUtm(LatLong.valueOf(center.getLatitude(),
										center.getLongitude(), NonSI.DEGREE_ANGLE), ReferenceEllipsoid.WGS84);
						newBoat = new SimulatedBoat(boat_name, initial_simulated_utm);
				}


				available_boats_spinner_adapter.add(boat_name);
				available_boats_spinner_adapter.notifyDataSetChanged();

				// marker view - automatically generate colored arrow
				Drawable arrow = getResources().getDrawable(R.drawable.arrow_white, null);
				PorterDuff.Mode mMode = PorterDuff.Mode.MULTIPLY;
				int boat_color = color_map.get(boat_color_count).get("boat");
				int line_color = color_map.get(boat_color_count).get("line");
				//arrow.setColorFilter(boat_color, mMode);
				newBoat.setBoatColor(boat_color);
				newBoat.setLineColor(line_color);
				boat_color_count++; // use the next set of colors
				if (boat_color_count > 5) boat_color_count = 0; // have a finite set of defined colors

				boat_markers_map.put(boat_name, new MarkerViewOptions()
								.position(pHollowStartingPoint)
								.title(boat_name)
								.icon(colorIconFromDrawable(arrow, boat_color))
								.rotation(0)
								.anchor(0.5f, 0.5f)
								.flat(true));

				// try to add the marker until mMapboxMap exists and it is added
				uiHandler.post(new Runnable()
				{
						@Override
						public void run()
						{
								if (mMapboxMap != null)
								{
										Log.i(logTag, String.format("Adding boat marker for %s", boat_name));
										mMapboxMap.addMarker(boat_markers_map.get(boat_name));
										marker_types_map.put(boat_markers_map.get(boat_name).getTitle(),
														PlatypusMarkerTypes.VEHICLE);
								}
								else
								{
										uiHandler.postDelayed(this, 1000);
								}
						}
				});

				// Path
				path_map.put(boat_name, new Path());
				waypath_outline_map.put(boat_name, new ArrayList<Polyline>());
				waypath_top_map.put(boat_name, new ArrayList<Polyline>());

				// waypoint indices
				current_wp_index_map.put(boat_name, -1);
				old_wp_index_map.put(boat_name, -2);

				newBoat.createListeners(
								new BoatMarkerUpdateRunnable(newBoat),
								new SensorDataReceivedRunnable(newBoat),
								new WaypointStateReceivedRunnable(newBoat),
								new CrumbReceivedRunnable(newBoat),
								new RCOverrideUpdateRunnable(newBoat));
				boats_map.put(boat_name, newBoat);
		}
		class BoatMarkerUpdateRunnable implements Runnable
		{
				Boat boat;
				String name;
				MarkerView marker_view;
				long last_redraw = 0;
				public BoatMarkerUpdateRunnable(Boat _boat)
				{
						boat = _boat;
						name = boat.getName();
						marker_view = boat_markers_map.get(name).getMarker();
				}
				public void run()
				{
						if (marker_view == null) return;
						marker_view.setPosition(boat.getLocation());
						float degree = (float) (boat.getYaw() * 180 / Math.PI);  // degree is -90 to 270
						degree = (degree < 0 ? 360 + degree : degree); // degree is 0 to 360
						marker_view.setVisible(true);
						//Log.d(logTag, "BoatMarkerUpdateRunnable: \n" +
						//				String.format("%s, yaw = %f, isVisible = %s", name, degree, Boolean.toString(marker_view.isVisible())));
						marker_view.setRotation(degree);

						// boat to current waypoint line
						//Log.d(logTag, String.format("boat-to-wp last redraw is %d ms ago", System.currentTimeMillis() - last_redraw));
						/*
						if (System.currentTimeMillis() - last_redraw < 500) return; // don't update the line too often
						Polyline line = boat_to_wp_line_map.get(name);
						Path path = path_map.get(name);
						if (line != null)
						{
								mMapboxMap.removeAnnotation(line);
								line.remove();
						}
						if (path == null) return;
						ArrayList<ArrayList<LatLng>> point_pairs = path.getPointPairs();
						//Log.d(logTag, String.format("current index = %d,  point_pairs.size() = %d", current_wp_index_map.get(name), point_pairs.size()));
						if (current_wp_index_map.get(name) < 0 || point_pairs.size() < 1) return;
						//Log.d(logTag, String.format("redrawing boat-to-wp line for %s", name));
						ArrayList<LatLng> pair = new ArrayList<>();
						pair.add(boat.getLocation());
						pair.add(point_pairs.get(current_wp_index_map.get(name)).get(0));
						boat_to_wp_line_map.put(name, mMapboxMap.addPolyline(new PolylineOptions().addAll(pair).color(boat.getBoatColor()).width(1)));
						last_redraw = System.currentTimeMillis();
						*/
				}
		}

		class SensorDataReceivedRunnable implements Runnable
		{
				Boat boat;
				String name;
				SensorData lastReceived;
				SensorDataReceivedRunnable(Boat _boat)
				{
						boat = _boat;
						name = boat.getName();
				}
				public void run()
				{
						lastReceived = boat.getLastSensorDataReceived();
						Log.v("SensorStuff", String.format("New SensorData received for %s: %s", name, lastReceived.toString()));
						// is the boat with this listener the selected boat?
						String selected_boat_name = available_boats_spinner.getSelectedItem().toString();
						if (name.equals(selected_boat_name))
						{
								// battery gets special treatment
								if (lastReceived.type == VehicleServer.DataType.BATTERY)
								{
										battery_value.setText(Double.toString(lastReceived.value) + " V");
								}
								else
								{
										sensor_stuff.newSensorData(lastReceived);
								}
						}
						/*
						// update the sensor text
						// TODO: generate a quasi-heatmap using GeoJSON collections and blurred circles
						// TODO: https://github.com/mapbox/mapbox-android-demo/blob/master/MapboxAndroidDemo/src/main/java/com/mapbox/mapboxandroiddemo/examples/dds/CreateHeatmapPointsActivity.java
						*/
				}
		}

		class WaypointStateReceivedRunnable implements Runnable
		{
				Boat boat;
				String name;
				WaypointStateReceivedRunnable(Boat _boat)
				{
						boat = _boat;
						name = boat.getName();
				}
				public void run()
				{
						// only display the currently selected boat's WP status
						String waypointState = boat.getWaypointState();
						Object result = available_boats_spinner.getSelectedItem();
						String current_boat_name = result.toString();
						if (name.equals(current_boat_name))
						{
								waypointInfo.setText(waypointState);
						}
				}
		}

		class CrumbReceivedRunnable implements Runnable
		{
				Boat boat;
				String name;
				LatLng crumb;
				Icon icon;
				CrumbReceivedRunnable(Boat _boat)
				{
						Log.i("ODE", "CrumbReceivedRunnable constructor");
						boat = _boat;
						name = boat.getName();
						crumb_markers_map.put(name, new ArrayList<Marker>());
						icon = colorIconFromDrawable(
										getResources().getDrawable(R.drawable.breadcrumb_pin, null),
										_boat.getBoatColor());
				}
				public void run()
				{
						crumb = boat.getNewCrumb();
						int size = crumb_markers_map.get(name).size();
						String title = "crumb_" + Integer.toString(size);
						crumb_markers_map.get(name).add(mMapboxMap.addMarker(new MarkerOptions().position(crumb).icon(icon).title(title)));
						marker_types_map.put(title, PlatypusMarkerTypes.BREADCRUMB);
				}
		}

		class RCOverrideUpdateRunnable implements Runnable
		{
				Boat boat;
				String name;
				RCOverrideUpdateRunnable(Boat _boat)
				{
						boat = _boat;
						name = boat.getName();
				}
				@Override
				public void run()
				{
						boolean rc_override_on = boat.isRCOverrideOn();
						if (rc_override_on)
						{
								// make big red text appear at the top of the app
								rc_override_warning.setText("RC OVERRIDE IS ON");
								rc_override_warning.setTextColor(Color.RED);
								rc_override_warning.setTextSize(30);
						}
						else
						{
								// make sure no big red text is showing
								rc_override_warning.setText("");
						}
				}
		}

		class TabletLocationMarkerRunnable implements Runnable
		{
				@Override
				public void run()
				{
						double lat;
						double lon;
						synchronized (tablet_location_lock)
						{
								lat = tablet_latlng[0];
								lon = tablet_latlng[1];
						}

						if (mMapboxMap == null) return;

						MarkerView marker_view = tablet_location_markerviewoptions.getMarker();
						if (marker_view == null) return;
						Log.d(logTag, "Updating operator marker location");
						marker_view.setPosition(new LatLng(lat, lon));
						marker_view.setVisible(true);
				}
		}

		class LoadedWaypointsRunnable implements Runnable
		{
				// Custom runnable that owns a list of waypoints, used for loading waypoints from a file
				private ArrayList<LatLng> waypoints;
				private LoadedWaypointsRunnable(ArrayList<LatLng> _waypoints)
				{
						waypoints = _waypoints;
				}
				public void setWaypoints(ArrayList<LatLng> _waypoints)
				{
						waypoints = _waypoints;
				}
				@Override
				public void run()
				{
						if (waypoints != null) replaceWaypointMarkers(waypoints);
				}
		}

		Boat currentBoat()
		{
				Object result = available_boats_spinner.getSelectedItem();
				if (result == null) return null;
				String boat_name = result.toString();
				return boats_map.get(boat_name);
		}

		class ToastFailureCallback implements Runnable
		{
				private String toastString;
				public ToastFailureCallback(String _toastString) { toastString = _toastString; }
				@Override
				public void run() { Toast.makeText(context, toastString, Toast.LENGTH_SHORT).show(); }
		}

		void addSingleWaypointMarker(LatLng point)
		{
				waypoint_list.add(point);
				String title = "waypoint_" + Integer.toString(marker_list.size());
				marker_list.add(mMapboxMap.addMarker(new MarkerOptions().position(point).title(title)));
				marker_types_map.put(title, PlatypusMarkerTypes.WAYPOINT);
				Log.v(logTag, String.format("waypoint_list.size() = %d,   marker_list.size() = %d", waypoint_list.size(), marker_list.size()));
		}
		void addWaypointMarkers(ArrayList<LatLng> points)
		{
				for (LatLng wp : points)
				{
						addSingleWaypointMarker(wp);
				}
		}
		void clearWaypointMarkers()
		{
				unowned_path.clearPoints();
				removeWaypaths("");
				waypoint_list.clear();
				mMapboxMap.removeAnnotations(marker_list);
				marker_list.clear();
				calculatePathDistance();
		}
		void replaceWaypointMarkers(ArrayList<LatLng> new_points)
		{
				ArrayList<LatLng> temp = (ArrayList<LatLng>)new_points.clone();
				clearWaypointMarkers();
				addWaypointMarkers(temp);
		}

		protected void onCreate(final Bundle savedInstanceState)
		{
				super.onCreate(savedInstanceState);
				Mapbox.getInstance(getApplicationContext(), getString(R.string.mapbox_access_token));
				this.setContentView(R.layout.tabletlayoutswitch);
				sensor_stuff = new SensorStuff(this);
				saved_waypoint_stuff = new SavedWaypointsStuff(context);

				SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
				File logDirectory = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), "platypus");
				mLogFile = new File(logDirectory,"platypus_" + sdf.format(new Date()) + ".txt");
				try {
						logDirectory.mkdirs();
						mLogFile.createNewFile();
						mLogWriter = new PrintWriter(mLogFile);
						mLogStartTime = System.currentTimeMillis();
				} catch (IOException e) {
						Log.e(logTag, "Failed to create log file: " + mLogFile, e);
						return;
				}

				// establish color_map
				color_map.put(0, new HashMap<String, Integer>());
				color_map.put(1, new HashMap<String, Integer>());
				color_map.put(2, new HashMap<String, Integer>());
				color_map.put(3, new HashMap<String, Integer>());
				color_map.put(4, new HashMap<String, Integer>());
				color_map.put(5, new HashMap<String, Integer>());
				color_map.get(0).put("boat", ContextCompat.getColor(context, R.color.magenta_light));
				color_map.get(0).put("line", ContextCompat.getColor(context, R.color.magenta_dark));
				color_map.get(1).put("boat", ContextCompat.getColor(context, R.color.cyan_light));
				color_map.get(1).put("line", ContextCompat.getColor(context, R.color.cyan_dark));
				color_map.get(2).put("boat", ContextCompat.getColor(context, R.color.orange_light));
				color_map.get(2).put("line", ContextCompat.getColor(context, R.color.orange_dark));
				color_map.get(3).put("boat", ContextCompat.getColor(context, R.color.green_light));
				color_map.get(3).put("line", ContextCompat.getColor(context, R.color.green_dark));
				color_map.get(4).put("boat", ContextCompat.getColor(context, R.color.red_light));
				color_map.get(4).put("line", ContextCompat.getColor(context, R.color.red_dark));
				color_map.get(5).put("boat", ContextCompat.getColor(context, R.color.yellow_light));
				color_map.get(5).put("line", ContextCompat.getColor(context, R.color.yellow_dark));

				linlay = (RelativeLayout) this.findViewById(R.id.linlay);
				ipAddressBox = (TextView) this.findViewById(R.id.printIpAddress);
				connect_button = (Button) this.findViewById(R.id.connectButton);
				start_wp_button = (Button) this.findViewById(R.id.start_button);
				pause_wp_button = (Button) this.findViewById(R.id.pause_button);
				stop_wp_button = (Button) this.findViewById(R.id.stop_button);
				battery_value = (TextView) this.findViewById(R.id.batteryVoltage);
				joystick = (JoystickView) findViewById(R.id.joystickView);
				transect_distance_input = (EditText) this.findViewById(R.id.transect_distance_input);
				waypointInfo = (TextView) this.findViewById(R.id.waypoint_status);
				path_length_value = (TextView) this.findViewById(R.id.path_length_value);

				advanced_options_button = (Button) this.findViewById(R.id.advopt);
				center_on_boat_button = (Button) this.findViewById(R.id.center_on_boat_button);
				center_on_operator_button = (Button) this.findViewById(R.id.center_on_operator_button);
				undo_last_wp_button = (Button) this.findViewById(R.id.undo_last_wp_button);
				remove_all_wp_button = (Button) this.findViewById(R.id.remove_all_wp_button);
				drop_wp_button = (Button) this.findViewById(R.id.drop_wp_button);
				normal_path_button = (Button) this.findViewById(R.id.path_button);
				spiral_button = (Button) this.findViewById(R.id.spiral_button);
				lawnmower_button = (Button) this.findViewById(R.id.lawnmower_button);
				reverse_order_button = (Button) this.findViewById(R.id.reverse_order_button);
				simple_path_button = (Button) this.findViewById(R.id.simple_path_button);

				jar1_button = (Button) this.findViewById(R.id.jar1_button);
				jar2_button = (Button) this.findViewById(R.id.jar2_button);
				jar3_button = (Button) this.findViewById(R.id.jar3_button);
				jar4_button = (Button) this.findViewById(R.id.jar4_button);
				sampler_reset_button = (Button) this.findViewById(R.id.sampler_reset_button);
				sampler_stop_all_button = (Button) this.findViewById(R.id.sampler_stop_all_button);
				jar1_button.setClickable(true);
				jar2_button.setClickable(true);
				jar3_button.setClickable(true);
				jar4_button.setClickable(true);
				jar1_text = (TextView) this.findViewById(R.id.jar1_text);
				jar2_text = (TextView) this.findViewById(R.id.jar2_text);
				jar3_text = (TextView) this.findViewById(R.id.jar3_text);
				jar4_text = (TextView) this.findViewById(R.id.jar4_text);
				jar1_text.setTextIsSelectable(false);
				jar2_text.setTextIsSelectable(false);
				jar3_text.setTextIsSelectable(false);
				jar4_text.setTextIsSelectable(false);

				rc_override_warning = (TextView) this.findViewById(R.id.rc_override_warning);

				alarm_ringtone = RingtoneManager.getRingtone(getApplicationContext(), alarmUri);
				notificationManager = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);

				speed_spinner_erroneous_call = true; // reset the erroneous call boolean
				speed_spinner = (Spinner) this.findViewById(R.id.speed_spinner);
				set_speed_spinner_from_pref();

				available_boats_spinner = (Spinner) this.findViewById(R.id.boat_name_spinner);
				available_boats_spinner_adapter = new ColorfulSpinnerAdapter(this, R.layout.boat_name);
				available_boats_spinner_adapter.setDropDownViewResource(R.layout.boat_name);
				available_boats_spinner.setAdapter(available_boats_spinner_adapter);
				available_boats_spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener()
				{
						@Override
						public void onItemSelected(AdapterView<?> parent, View view, int position, long id)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "available_boats_spinner");
								Toast.makeText(getApplicationContext(),
												String.format(
																"Controlling: %s",
																available_boats_spinner.getSelectedItem().toString()),
												Toast.LENGTH_SHORT
								).show();
								Boat boat = currentBoat();
								if (boat != null)
								{
										ipAddressBox.setText(boat.getIpAddressString());
										// reset the other status elements, let the listeners update them
										sensor_stuff.newSensorSet(); // clear the sensor text data
										waypointInfo.setText("");
										battery_value.setText("");

										current_boat_is_connected.set(boat.isConnected());
								}
						}

						@Override
						public void onNothingSelected(AdapterView<?> parent)
						{

						}
				});

				alarm_ringtone = RingtoneManager.getRingtone(getApplicationContext(), alarmUri);
				notificationManager = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);

				SettingsActivity.set_TeleOpPanel(this);
				loadPreferences();
				battery_value.setText("");

				//Create folder for the first time if it does not exist
				final File waypointDir = new File(Environment.getExternalStorageDirectory() + "/waypoints");
				if (!waypointDir.exists())
				{
						waypointDir.mkdir();
				}

				mv = (MapView) findViewById(R.id.mapview);
				mv.onCreate(savedInstanceState);
				mv.getMapAsync(new OnMapReadyCallback()
				{
						@Override
						public void onMapReady(@NonNull final MapboxMap mapboxMap)
						{
								Log.i(logTag, "mapboxmap ready");
								mMapboxMap = mapboxMap;
								if (setInitialPan == true && initialPan.getLatitude() != 0 || initialPan.getLongitude() != 0)
								{
										mMapboxMap.moveCamera(CameraUpdateFactory.newCameraPosition(
														new CameraPosition.Builder()
																		.target(initialPan)
																		.zoom(16)
																		.build()
										));
								}

								mMapboxMap.setStyle(Style.MAPBOX_STREETS); //vector map
								mMapboxMap.getUiSettings().setRotateGesturesEnabled(false);
								mMapboxMap.setAllowConcurrentMultipleOpenInfoWindows(true);
								mMapboxMap.setInfoWindowAdapter(new MapboxMap.InfoWindowAdapter()
								{
										@Override
										public View getInfoWindow(final Marker marker)
										{
												String title = marker.getTitle();
												// TODO: display different information based on marker type
												if (marker_types_map.get(title) == PlatypusMarkerTypes.WAYPOINT)
												{
														View view = getLayoutInflater().inflate(R.layout.waypoint_info_window, null);
														TextView waypoint_index_textview = (TextView) view.findViewById(R.id.waypoint_index_textview);
														TextView waypoint_latlng_textview = (TextView) view.findViewById(R.id.waypoint_latlong_textview);
														waypoint_latlng_textview.setText(String.format("%s, %s",
																		marker.getPosition().getLatitude(),
																		marker.getPosition().getLongitude()));

														// parse waypoint title to get its index
														String[] title_parts = title.split("_");
														final int waypoint_index = Integer.valueOf(title_parts[1]);
														final Button move_button = (Button) view.findViewById(R.id.waypoint_move_button);
														move_button.setOnClickListener(new OnClickListener()
														{
																@Override
																public void onClick(View v)
																{
																		mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "waypoint_move");
																		// next map click sets the marker's location and resets the map click listener to null
																		mMapboxMap.setOnMapClickListener(new MapboxMap.OnMapClickListener()
																		{
																				@Override
																				public void onMapClick(@NonNull LatLng point)
																				{
																						marker.setPosition(point);
																						waypoint_list.set(waypoint_index, point);
																						mMapboxMap.setOnMapClickListener(new MapboxMap.OnMapClickListener()
																						{
																								@Override
																								public void onMapClick(@NonNull LatLng point) { }
																						});
																				}
																		});
																}
														});
														waypoint_index_textview.setText(marker.getTitle());
														return view;
												}
												else
												{
														return null;
												}
										}
								});
								mMapboxMap.setOnMarkerClickListener(new MapboxMap.OnMarkerClickListener()
								{
										@Override
										public boolean onMarkerClick(@NonNull Marker marker)
										{
												return false;
										}
								});
								mMapboxMap.setOnMapLongClickListener(new MapboxMap.OnMapLongClickListener()
								{
										@Override
										public void onMapLongClick(LatLng point)
										{
												mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "added_new_waypoint");
												addSingleWaypointMarker(point);
										}
								});
								mIconFactory = IconFactory.getInstance(context);
						}
				});

				// Every second, display boat connection status
				uiHandler.post(new Runnable()
				{
						@Override
						public void run()
						{
								final Boat boat = currentBoat();
								if (boat != null)
								{
										boolean isConnected = boat.isConnected();

										if (isConnected != current_boat_is_connected.get())
										{
												// TODO: status has changed, log event
												String status;
												if (isConnected)
												{
														status = "connected";
												}
												else
												{
														status = "disconnected";
												}
												Log.w(logTag, String.format("Connection to \"%s\" changed, now %s", boat.getName(), status));

												// TODO: log may use the old position of the boat on a re-connection if the poseListener hasn't received a new pose
												// TODO: how can we ensure that the updated position is used?
												// TODO: we could delay the log statement by a little bit
												final String log_string = System.currentTimeMillis()-mLogStartTime + "\t" + boat.getName() + " " + status;
												uiHandler.postDelayed(new Runnable()
												{
														@Override
														public void run()
														{
																String log_string_copy = log_string;
																if (tablet_gps_fix)
																{
																		LatLng operator_location;
																		synchronized (tablet_location_lock)
																		{
																				operator_location = new LatLng(tablet_latlng[0], tablet_latlng[1]);
																		}
																		double distance_to_boat = boat.distanceFromOperator(operator_location);
																		log_string_copy += "\t" + String.format(Locale.US, "distance = %.1f", distance_to_boat);
																}
																mLogWriter.println(log_string_copy);
														}
												}, 1000);
										}
										current_boat_is_connected.set(isConnected);

										if (isConnected)
										{
												ipAddressBox.setBackgroundColor(Color.GREEN);
										}
										else
										{
												ipAddressBox.setBackgroundColor(Color.RED);
										}
								}
								else
								{
										ipAddressBox.setBackgroundColor(Color.RED);
								}
								uiHandler.postDelayed(this, 500);
						}
				});

				center_on_boat_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View view)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "center_on_boat");
								if (mMapboxMap == null)
								{
										Toast.makeText(getApplicationContext(), "Please wait for the map to load", Toast.LENGTH_LONG).show();
										return;
								}
								Boat boat = currentBoat();
								if (boat == null)
								{
										Toast.makeText(getApplicationContext(), "Please Connect to a boat first", Toast.LENGTH_LONG).show();
										return;
								}
								LatLng location = currentBoat().getLocation();
								if (location == null)
								{
										Toast.makeText(getApplicationContext(), "Boat still finding GPS location", Toast.LENGTH_LONG).show();
										return;
								}
								mMapboxMap.moveCamera(CameraUpdateFactory.newCameraPosition(new CameraPosition.Builder().target(location).zoom(16).build()));
						}
				});

				center_on_operator_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "center_on_operator");
								if (mMapboxMap == null)
								{
										Toast.makeText(getApplicationContext(), "Please wait for the map to load", Toast.LENGTH_LONG).show();
										return;
								}
								if (!tablet_gps_fix)
								{
										Toast.makeText(getApplicationContext(), "Tablet does not have GPS fix", Toast.LENGTH_SHORT).show();
										return;
								}
								double lat, lon;
								synchronized (tablet_location_lock)
								{
										lat = tablet_latlng[0];
										lon = tablet_latlng[1];
								}
								LatLng oploc = new LatLng(lat, lon);
								mMapboxMap.moveCamera(CameraUpdateFactory.newCameraPosition(new CameraPosition.Builder().target(oploc).zoom(16).build()));
						}
				});

				//Options menu
				advanced_options_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{

								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "advanced_options");

								PopupMenu popup = new PopupMenu(TeleOpPanel.this, advanced_options_button);
								popup.getMenuInflater().inflate(R.menu.dropdownmenu, popup.getMenu());
								popup.setOnMenuItemClickListener(new PopupMenu.OnMenuItemClickListener()
								{
										public boolean onMenuItemClick(MenuItem item)
										{
												switch (item.toString())
												{
														case "Satellite Map":
														{
																if (mMapboxMap != null) mMapboxMap.setStyle(Style.SATELLITE);
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "satellite_map");
																break;
														}
														case "Vector Map":
														{
																if (mMapboxMap != null) mMapboxMap.setStyle(Style.MAPBOX_STREETS);
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "vector_map");
																break;
														}
														case "Set Home":
														{
																setHome();
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "set_home");
																break;
														}
														case "Go Home":
														{
																goHome();
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "go_home");
																break;
														}
														case "Send PIDs":
														{
																sendPID();
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "send_PIDs");
																break;
														}
														case "Save Waypoints":
														{
																if (waypoint_list.size() < 1)
																{
																		Toast.makeText(context, "Need at least 1 wp first", Toast.LENGTH_SHORT).show();
																		break;
																}
																saved_waypoint_stuff.saveWaypointsToFile(waypoint_list);
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "save_waypoints");
																break;
														}
														case "Load Waypoints":
														{
																saved_waypoint_stuff.loadWaypointsFromFile(new LoadedWaypointsRunnable(new ArrayList<LatLng>()));
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "load_waypoints");
																break;
														}
														case "Snooze Alarms":
														{
																synchronized (_batteryVoltageLock)
																{
																		sleep_start_time = System.currentTimeMillis();
																		alarm_on = false;
																		alarm_ringtone.stop();
																}
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "snooze_alarms");
																break;
														}
														case "Preferences":
														{
																Intent intent = new Intent(context, SettingsActivity.class);
																context.startActivity(intent);
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "preferences");
																break;
														}
														case "Autonomy":
														{
																Intent intent = new Intent(context, AutonomyActivity.class);
																context.startActivity(intent);
																mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "autonomy");
																break;
														}
												}
												return true;
										}
								});
								popup.show();
						}
				});

				speed_spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener()
				{
						@Override
						public void onItemSelected(AdapterView<?> parent, View view, int position, long id)
						{
								// this listener triggers when the view is initialized, no user touch event
								// That causes undesired behavior, so we need to make sure it is ignored once
								if (speed_spinner_erroneous_call)
								{
										speed_spinner_erroneous_call = false;
										return;
								}
								SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
								SharedPreferences.Editor editor = sharedPref.edit();
								String item = String.valueOf(speed_spinner.getSelectedItem());
								switch (item)
								{
										case "Slow":
												editor.putString(SettingsActivity.KEY_PREF_SPEED, "SLOW");
												break;
										case "Medium":
												editor.putString(SettingsActivity.KEY_PREF_SPEED, "MEDIUM");
												break;
										case "Fast":
												editor.putString(SettingsActivity.KEY_PREF_SPEED, "FAST");
												break;
										case "Custom":
												editor.putString(SettingsActivity.KEY_PREF_SPEED, "CUSTOM");
												break;
										default:
												break;
								}
								editor.apply();
								editor.commit();

								Toast.makeText(getApplicationContext(), String.valueOf(speed_spinner.getSelectedItem()), Toast.LENGTH_SHORT).show();
								sendPID();

								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "speed_spinner" + "\t" + item);
						}

						@Override
						public void onNothingSelected(AdapterView<?> parent)
						{

						}
				});

				// Joystick
				joystick.setYAxisInverted(false);
				joystick.setOnJostickMovedListener(joystick_moved_listener);
				joystick.setOnJostickClickedListener(null);

				senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
				senAccelerometer = senSensorManager
								.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
				senSensorManager.registerListener(this, senAccelerometer,
								SensorManager.SENSOR_DELAY_NORMAL);

				connect_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "connect_to_boat");
								connectBox();
						}
				});
				connectBox(); // start the app with the connect dialog popped up

				start_wp_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "start_waypoints");
								Log.i(logTag, "startWaypoints() called...");
								paused = false;
								pause_wp_button.setBackground(getDrawable(R.drawable.pause_button));
								Boat boat = currentBoat();
								if (boat == null)
								{
										Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
										Log.w(logTag, "TeleOpPanel.startWaypoints(): currentBoat is null");
										return;
								}
								String boat_name = boat.getName();
								// if there are no waypoints, the user has to create waypoints, then a path
								if (waypoint_list.size() < 1)
								{
										Toast.makeText(context, "Create waypoints and a path first", Toast.LENGTH_SHORT).show();
										return;
								}
								// if there is exactly one waypoint, create a "path" for the user
								if (waypoint_list.size() == 1)
								{
										unowned_path = new Path((ArrayList<LatLng>) waypoint_list.clone());
								}
								// if there are no points in unowned_path, the user has to create a path
								if (unowned_path.getPoints().size() < 1)
								{
										Toast.makeText(context, "Create a path first", Toast.LENGTH_SHORT).show();
										return;
								}

								ArrayList<LatLng> points = (ArrayList<LatLng>)unowned_path.getPoints().clone();
								path_map.put(boat_name, new Path(points));
								double[][] waypoints = new double[points.size()][2];
								for (int i = 0; i < points.size(); i++)
								{
										LatLng latlng = points.get(i);
										waypoints[i] = new double[] {latlng.getLatitude(), latlng.getLongitude()};
								}

								boat.startWaypoints(waypoints, new ToastFailureCallback("Start Waypoints Msg Timed Out"));
								current_wp_index_map.put(boat_name, 0);

								// draw the boat's lines, independent from the ones used to generate paths
								removeWaypaths(boat_name);
								addWaypaths(boat_name);

								// Here is where you'd clear the waypoints_list, marker_list, and clear the unowned path
								// But you might want to give the exact same path to another boat, so I'll leave it to the user to clear
						}
				});

				pause_wp_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "pause_waypoints");
								Boat boat = currentBoat();
								if (boat != null)
								{
										if (!paused) // pause
										{
												boat.setAutonomous(false, new ToastFailureCallback("Pause Msg Timed Out"));
												pause_wp_button.setBackground(getDrawable(R.drawable.resume_button));
										}
										else // unpause
										{
												boat.setAutonomous(true, new ToastFailureCallback("Resume Msg Timed Out"));
												pause_wp_button.setBackground(getDrawable(R.drawable.pause_button));
										}
										paused = !paused;

								}
								// TODO: how can we implement resume with boat state rather than button state?
						}
				});

				stop_wp_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "stop_waypoints");
								paused = false;
								pause_wp_button.setBackground(getDrawable(R.drawable.pause_button));
								Boat boat = currentBoat();
								if (boat != null)
								{
										String boat_name = boat.getName();
										boat.stopWaypoints(new ToastFailureCallback("Stop Waypoints Msg Timed Out"));
										removeWaypaths(boat_name);
										path_map.get(boat_name).clearPoints();
										waypath_outline_map.get(boat_name).clear();
										waypath_top_map.get(boat_name).clear();
										//clearWaypointMarkers();
								}
						}
				});

				undo_last_wp_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "undo_last_waypoint");
								Log.i(logTag, String.format("waypoint_list.size() = %d,   marker_list.size() = %d", waypoint_list.size(), marker_list.size()));
								if (marker_list.size() > 0)
								{
										mMapboxMap.removeAnnotation(marker_list.get(marker_list.size()-1));
										waypoint_list.remove(waypoint_list.size() - 1);
										marker_list.remove(marker_list.size() - 1);
										unowned_path.clearPoints();
										removeWaypaths("");
								}
								else
								{
										Toast.makeText(context, "No more waypoints", Toast.LENGTH_SHORT).show();
								}
						}
				});

				remove_all_wp_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "remove_all_waypoints");
								Log.i(logTag, String.format("waypoint_list.size() = %d,   marker_list.size() = %d", waypoint_list.size(), marker_list.size()));
								if (marker_list.size() > 0)
								{
										clearWaypointMarkers();
								}
						}
				});

				drop_wp_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "drop_waypoint_on_boat");
								final Boat boat = currentBoat();
								if (boat == null)
								{
										Toast.makeText(getApplicationContext(), "No Boat Connected", Toast.LENGTH_LONG).show();
										return;
								}
								if (boat.getLocation() == null)
								{
										Toast.makeText(getApplicationContext(), "Waiting on boat GPS", Toast.LENGTH_LONG).show();
										return;
								}
								LatLng point = boat.getLocation();
								if (mMapboxMap == null)
								{
										Toast.makeText(getApplicationContext(), "Map still loading", Toast.LENGTH_LONG).show();
										return;
								}
								addSingleWaypointMarker(point);
						}
				});

				normal_path_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "normal_path");
								// create a Path object from the current waypoints
								// draw the lines between the current waypoints to show the path
								// calculate the path length and show it in the text
								unowned_path.clearPoints();
								removeWaypaths("");
								Log.v(logTag, String.format("waypoint_list.size() = %d,   marker_list.size() = %d", waypoint_list.size(), marker_list.size()));
								if (waypoint_list.size() > 0)
								{
										unowned_path = new Path((ArrayList<LatLng>)waypoint_list.clone());
										addWaypaths("");
										calculatePathDistance();
								}
								else
								{
										Toast.makeText(context, "Need waypoints to generate path", Toast.LENGTH_SHORT).show();
								}
						}
				});

				spiral_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "spiral_path");
								try
								{
										currentTransectDist = Double.valueOf(transect_distance_input.getText().toString());
								}
								catch (Exception ex)
								{
										// user probably has a bad transect distance typed in
										Toast.makeText(context, "Strange transect distance. Using 10.", Toast.LENGTH_SHORT).show();
										transect_distance_input.setText("10");
										currentTransectDist = 10;
								}
								unowned_path.clearPoints();
								removeWaypaths("");
								if (waypoint_list.size() > 2)
								{
										try
										{
												Region region = new Region((ArrayList<LatLng>) waypoint_list.clone(), AreaType.SPIRAL, currentTransectDist);
												unowned_path = region.convertToSimplePath();
										}
										catch (Exception e)
										{
												Log.e(logTag, String.format("Generating spiral error: %s", e.getMessage()));
												return;
										}
										addWaypaths("");
										calculatePathDistance();
								}
								else
								{
										Toast.makeText(context, "Need 3 waypoints to generate spiral", Toast.LENGTH_SHORT).show();
								}
						}
				});

				lawnmower_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "lawnmower_path");
								try
								{
										currentTransectDist = Double.valueOf(transect_distance_input.getText().toString());
								}
								catch (Exception ex)
								{
										// user probably has a bad transect distance typed in
										Toast.makeText(context, "Strange transect distance. Using 20.", Toast.LENGTH_SHORT).show();
										transect_distance_input.setText("20");
										currentTransectDist = 20;
								}
								unowned_path.clearPoints();
								removeWaypaths("");
								if (waypoint_list.size() > 2)
								{
										try
										{
												Region region = new Region((ArrayList<LatLng>) waypoint_list.clone(), AreaType.LAWNMOWER, currentTransectDist);
												unowned_path = region.convertToSimplePath();
										}
										catch (Exception e)
										{
												Log.e(logTag, String.format("Generating lawnmower error: %s", e.getMessage()));
												return;
										}
										addWaypaths("");
										calculatePathDistance();
								}
								else
								{
										Toast.makeText(context, "Need 3 waypoints to generate lawnmower", Toast.LENGTH_SHORT).show();
								}
						}
				});

				reverse_order_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "reverse_waypoints_order");
								if (waypoint_list.size() > 1)
								{
										unowned_path.clearPoints();
										removeWaypaths("");
										Collections.reverse(waypoint_list);
										ArrayList<LatLng> temp = (ArrayList<LatLng>)waypoint_list.clone(); // shallow copy
										replaceWaypointMarkers(temp);
								}
						}
				});

				simple_path_button.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "convert_to_normal_path");
								if (unowned_path.getPoints().size() > 2)
								{
										replaceWaypointMarkers(unowned_path.getPoints());
								}
								else
								{
										Toast.makeText(context, "Generate a path with at least 3 waypoints first", Toast.LENGTH_SHORT).show();
								}
						}
				});

				class JarCountdownTimer extends CountDownTimer
				{
						TextView clock_text;
						int seconds;
						int minutes;
						int full_seconds;
						int left_seconds;

						public JarCountdownTimer(long durationMillis, long tickIntervalMillis, TextView jar_textview)
						{
								super(durationMillis, tickIntervalMillis);
								clock_text = jar_textview;
						}

						public void onTick(long millisUntilFinished)
						{
								seconds = (int)millisUntilFinished/1000;
								minutes = seconds/60;
								full_seconds = minutes*60;
								left_seconds = seconds - full_seconds;
								if (left_seconds < 10)
								{
										clock_text.setText(String.format("%d:0%d", minutes, left_seconds));
								}
								else
								{
										clock_text.setText(String.format("%d:%d", minutes, left_seconds));
								}

						}

						public void onFinish()
						{
								clock_text.setText("DONE");
						}
				}
				class JarCountdownRunnable implements Runnable
				{
						TextView clock_text;
						Button button;
						CountDownTimer timer;
						public JarCountdownRunnable(TextView jar_textview, Button jar_button)
						{
								clock_text = jar_textview;
								button = jar_button;
						}
						@Override
						public void run()
						{
								timer = new JarCountdownTimer(1000*60*4, 1000, clock_text).start();
								button.setClickable(false);
						}

						public void cancel() { if (timer != null) timer.cancel(); }
				}
				class JarOnClickListener implements OnClickListener
				{
						TextView clock_text;
						Button button;
						int number;
						JarCountdownRunnable runnable;
						public JarOnClickListener(int jar_number, TextView jar_textview, Button jar_button)
						{
								clock_text = jar_textview;
								button = jar_button;
								number = jar_number;
						}
						@Override
						public void onClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "sampler_start_jar_" + Integer.toString(number));
								if (button.isClickable())
								{
										Boat boat = currentBoat();
										if (boat == null)
										{
												Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
												return;
										}
										runnable = new JarCountdownRunnable(clock_text, button);
										boat.startSample(number-1, runnable,
														new ToastFailureCallback("Sampler Start Msg timed out"));
								}
								else
								{
										// TODO: why does this not run?
										Toast.makeText(context, String.format("Jar %d is not available until reset", number), Toast.LENGTH_SHORT).show();
								}
						}
						public void cancel()
						{
								// cancel the timer inside the countdown runnable
								if (runnable != null) runnable.cancel();
						}
				}

				final JarOnClickListener jar1_listener = new JarOnClickListener(1, jar1_text, jar1_button);
				final JarOnClickListener jar2_listener = new JarOnClickListener(2, jar2_text, jar2_button);
				final JarOnClickListener jar3_listener = new JarOnClickListener(3, jar3_text, jar3_button);
				final JarOnClickListener jar4_listener = new JarOnClickListener(4, jar4_text, jar4_button);
				jar1_button.setOnClickListener(jar1_listener);
				jar2_button.setOnClickListener(jar2_listener);
				jar3_button.setOnClickListener(jar3_listener);
				jar4_button.setOnClickListener(jar4_listener);

				class ResetSamplerSuccessRunnable implements Runnable
				{
						@Override
						public void run()
						{
								jar1_button.setClickable(true);
								jar2_button.setClickable(true);
								jar3_button.setClickable(true);
								jar4_button.setClickable(true);
								jar1_listener.cancel();
								jar2_listener.cancel();
								jar3_listener.cancel();
								jar4_listener.cancel();
								jar1_text.setText("");
								jar2_text.setText("");
								jar3_text.setText("");
								jar4_text.setText("");

								Toast.makeText(context, "Sampler reset or stopped successfully", Toast.LENGTH_SHORT).show();
						}
				}
				sampler_reset_button.setOnLongClickListener(new View.OnLongClickListener()
				{
						@Override
						public boolean onLongClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "sampler_reset");
								Boat boat = currentBoat();
								if (boat == null)
								{
										Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
										return false;
								}
								boat.resetSampler(new ResetSamplerSuccessRunnable(),
												new ToastFailureCallback("Sampler Reset Msg timed out"));
								return false;
						}
				});
				sampler_stop_all_button.setOnLongClickListener(new View.OnLongClickListener()
				{
						@Override
						public boolean onLongClick(View v)
						{
								mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "button" + "\t" + "sampler_stop");
								Boat boat = currentBoat();
								if (boat == null)
								{
										Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
										return false;
								}
								boat.stopSampleAll(new ResetSamplerSuccessRunnable(),
												new ToastFailureCallback("Sampler Stop All Msg timed out"));
								return false;
						}
				});

				location_listener = new LocationListener()
				{
						@Override
						public void onLocationChanged(Location location)
						{
								// Convert from lat/long to UTM coordinates
								double lat = location.getLatitude();
								double lon = location.getLongitude();
								synchronized (tablet_location_lock)
								{
										tablet_latlng[0] = lat;
										tablet_latlng[1] = lon;
								}
								Log.d(logTag, String.format("tablet latlng = %f, %f", lat, lon));

								if (!tablet_gps_fix)
								{
										Log.i(logTag, "Creating tablet location marker");
										tablet_location_markerviewoptions = new MarkerViewOptions()
														.position(new LatLng(lat, lon))
														.title("operator")
														.icon(mIconFactory.fromResource(R.drawable.userloc))
														.rotation(0)
														.anchor(0.5f, 0.5f)
														.flat(true);
										mMapboxMap.addMarker(tablet_location_markerviewoptions);
								}
								tablet_gps_fix = true;

								uiHandler.post(new TabletLocationMarkerRunnable());
						}

						@Override
						public void onStatusChanged(String provider, int status, Bundle extras)  { }

						@Override
						public void onProviderEnabled(String provider) { }

						@Override
						public void onProviderDisabled(String provider) { }
				};

				LocationManager gps = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
				Criteria c = new Criteria();
				c.setAccuracy(Criteria.ACCURACY_FINE);
				c.setPowerRequirement(Criteria.NO_REQUIREMENT);
				String provider = gps.getBestProvider(c, false);
				gps.requestLocationUpdates(provider, 1000, 0, location_listener);
		}

		@Override
		protected void onStart()
		{
				super.onStart();
				mv.onStart();
		}

		@Override
		public void onResume()
		{
				super.onResume();
				mv.onResume();
		}

		@Override
		public void onPause()
		{
				super.onPause();
				mv.onPause();
		}

		@Override
		protected void onStop()
		{
				super.onStop();
				//mv.onStop();
		}

		@Override
		protected void onDestroy()
		{
				super.onDestroy();
				mv.onDestroy();

				SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
				SharedPreferences.Editor editor = sharedPref.edit();
				Boat boat = currentBoat();
				if (boat != null)
				{
						LatLng location = boat.getLocation();
						if (location != null)
						{
								editor.putString(SettingsActivity.KEY_PREF_LAT, Double.toString(location.getLatitude()));
								editor.putString(SettingsActivity.KEY_PREF_LON, Double.toString(location.getLongitude()));
						}
				}
				editor.apply();
				editor.commit();

				// Disconnect from GPS updates
				LocationManager gps;
				gps = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
				gps.removeUpdates(location_listener);

				if (mLogWriter != null) {
						mLogWriter.close();
						mLogWriter = null;
				}
		}

		@Override
		public void onLowMemory()
		{
				super.onLowMemory();
				mv.onLowMemory();
		}

		@Override
		protected void onSaveInstanceState(Bundle outState)
		{
				super.onSaveInstanceState(outState);
				mv.onSaveInstanceState(outState);
		}

		// This method checks the wifi connection but not Internet access
		public static boolean isNetworkAvailable(final Context context)
		{
				final ConnectivityManager connectivityManager = ((ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE));
				return connectivityManager.getActiveNetworkInfo() != null && connectivityManager.getActiveNetworkInfo().isConnectedOrConnecting();
		}

		// This method need to run in another thread except UI thread(main thread)
		public static boolean hasActiveInternetConnection(Context context)
		{
				if (isNetworkAvailable(context))
				{
						try
						{
								HttpURLConnection urlc = (HttpURLConnection) (new URL("http://www.google.com").openConnection());
								urlc.setRequestProperty("User-Agent", "Test");
								urlc.setRequestProperty("Connection", "close");
								urlc.setConnectTimeout(1500);
								urlc.connect();
								return (urlc.getResponseCode() == 200);
						}
						catch (IOException e)
						{
								Log.e(logTag, "Error checking internet connection", e);
						}
				}
				else
				{
						Log.d(logTag, "No network available!");
				}
				return false;
		}

		// *******************************
		//  JoystickView listener
		// *******************************
		Runnable joystickTimeoutCallback = new Runnable()
		{
				@Override
				public void run()
				{
						Toast.makeText(context, "Joystick Msg Timed Out!!", Toast.LENGTH_SHORT).show();
				}
		};
		private JoystickMovedListener joystick_moved_listener = new JoystickMovedListener()
		{
				Boat boat;
				@Override
				public void OnMoved(int x, int y)
				{
						Log.d(logTag, String.format("joystick (x, y) = %d, %d", x, y));
						boat = currentBoat();
						if (boat != null)
						{
								boat.updateControlSignals(
												fromProgressToRange(y, THRUST_MIN, THRUST_MAX),
												fromProgressToRange(x, RUDDER_MIN, RUDDER_MAX),
												joystickTimeoutCallback
								);
						}
				}

				@Override
				public void OnReleased() { }

				@Override
				public void OnReturnedToCenter()
				{
						mLogWriter.println(System.currentTimeMillis() - mLogStartTime + "\t" + "joystick");
						boat = currentBoat();
						if (boat != null)
						{
								boat.updateControlSignals(0.0, 0.0, joystickTimeoutCallback);
						}
				}
		};

		// Converts from progress bar value to linear scaling between min and max
		private double fromProgressToRange(int progress, double min, double max)
		{
				// progress will be between -10 and 10, with 0 being the center
				// evaluate linear range above and below zero separately
				double value;
				if (progress < 0)
				{
						value = min * Math.abs(progress) / 10.0;
						return value;
				}
				else
				{
						value = max * progress / 10.0;
						return value;
				}
		}

		/* accelerometer controls */
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) { }

		@Override
		public void onSensorChanged(SensorEvent sensorEvent)
		{
				Sensor mySensor = sensorEvent.sensor;
				if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) { }
		}

		public void connectBox()
		{
				// TODO: check if the IP address is already being used. If so, prompt to reconnect or just switch to the existing boat
				final Dialog dialog = new Dialog(context);
				dialog.setContentView(R.layout.connectdialog);
				dialog.setTitle("Connect To A Boat");
				ipAddressInput = (EditText) dialog.findViewById(R.id.ip_address_input);
				Button submitButton = (Button) dialog.findViewById(R.id.submit);
				real_boat_button = (RadioButton) dialog.findViewById(R.id.realBoatButton);
				sim_boat_button = (RadioButton) dialog.findViewById(R.id.simBoatButton);

				loadPreferences();
				ipAddressInput.setText(textIpAddress);

				real_boat_button.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener()
				{
						@Override
						public void onCheckedChanged(CompoundButton buttonView, boolean isChecked)
						{
								use_real_boat = isChecked;
						}
				});

				submitButton.setOnClickListener(new OnClickListener()
				{
						@Override
						public void onClick(View v)
						{

								if (ipAddressInput.getText() == null || ipAddressInput.getText().equals("") || ipAddressInput.getText().length() == 0)
								{
										ipAddressBox.setText("localhost");
								}
								else
								{
										ipAddressBox.setText(ipAddressInput.getText());
								}
								textIpAddress = ipAddressInput.getText().toString();
								InetSocketAddress address;
								if (ipAddressInput.getText() == null || ipAddressInput.getText().equals(""))
								{
										address = CrwNetworkUtils.toInetSocketAddress("127.0.0.1:" + boatPort);
								}
								else
								{
										address = CrwNetworkUtils.toInetSocketAddress(textIpAddress + ":" + boatPort);
								}
								int boat_count = boats_map.size();
								String boat_name = String.format("boat_%d", boat_count);
								// TODO: let the user choose the boat name
								startNewBoat(boat_name); // initialize the Boat
								boats_map.get(boat_name).setAddress(address);
								boats_map.get(boat_name).setIpAddressString(textIpAddress);
								available_boats_spinner.setSelection(boat_count); // automatically watch the new boat
								try
								{
										saveSession(); //save ip address
								}
								catch (Exception e) { }
								dialog.dismiss();
								latestWaypointPoll(); // Launch waypoint polling
								alertsAndAlarms(); // Launch alerts and alarms thread
								SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(getApplicationContext());
								SharedPreferences.Editor editor = sharedPref.edit();
								editor.putString(SettingsActivity.KEY_PREF_IP, boats_map.get(boat_name).getIpAddress().getAddress().toString());
								editor.apply();
								editor.commit();
						}
				});
				dialog.show();
		}

		//  Make return button same as home button
		@Override
		public void onBackPressed()
		{
				/*
				Intent setIntent = new Intent(Intent.ACTION_MAIN);
				setIntent.addCategory(Intent.CATEGORY_HOME);
				setIntent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
				startActivity(setIntent);
				*/
				// TODO: this causes a crash. Disabling the back button for now.
		}

		public void setHome()
		{
				// temporarily create new OnClickListener for map
				// new listener will set home for currently selected boat
				// then it will reset map's listener back to null
				// This is all just like the method used for the waypoint movement button
				final Boat boat = currentBoat();
				if (boat != null)
				{
						// next map click sets the marker's location and resets the map click listener to null
						mMapboxMap.setOnMapClickListener(new MapboxMap.OnMapClickListener()
						{
								@Override
								public void onMapClick(@NonNull final LatLng point)
								{

										Runnable successRunnable = new Runnable()
										{
												@Override
												public void run()
												{
														final String boat_name = boat.getName();

														// marker view - automatically generate colored arrow
														Drawable home = getResources().getDrawable(R.drawable.home_white, null);
														int boat_color = boat.getBoatColor();

														if (home_markers_map.containsKey(boat_name))
														{
																Log.i(logTag, String.format("clearing old home for boat \"%s\"", boat_name));
																mMapboxMap.removeAnnotation(home_markers_map.get(boat_name).getMarker());
																home_markers_map.remove(boat_name);
														}

														home_markers_map.put(boat_name, new MarkerViewOptions()
																		.position(point)
																		.title(boat_name + "_home")
																		.icon(colorIconFromDrawable(home, boat_color))
																		.rotation(0)
																		.anchor(0.5f, 0.5f)
																		.flat(true));

														// try to add the marker until mMapboxMap exists and it is added
														uiHandler.post(new Runnable()
														{
																@Override
																public void run()
																{
																		if (mMapboxMap != null)
																		{
																				Log.i(logTag, String.format("Adding home marker for %s", boat_name));
																				mMapboxMap.addMarker(home_markers_map.get(boat_name));
																				marker_types_map.put(home_markers_map.get(boat_name).getTitle(),
																								PlatypusMarkerTypes.HOME);
																		}
																		else
																		{
																				uiHandler.postDelayed(this, 1000);
																		}
																}
														});
												}
										};

										boat.setHome(point, successRunnable, new ToastFailureCallback("set home message timed out"));

										mMapboxMap.setOnMapClickListener(new MapboxMap.OnMapClickListener()
										{
												@Override
												public void onMapClick(@NonNull LatLng point) { }
										});
								}
						});
				}
				else
				{
						Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
				}
		}

		public void goHome()
		{
				final Boat boat = currentBoat();
				if (boat != null)
				{
						final LatLng home = boat.getHome();
						if (home == null)
						{
								Toast.makeText(getApplicationContext(), "Set home first!", Toast.LENGTH_SHORT).show();
								return;
						}
						new AlertDialog.Builder(context)
										.setTitle("Go Home")
										.setMessage("Force the boat to go home ?")
										.setPositiveButton("Yes", new DialogInterface.OnClickListener()
										{
												public void onClick(DialogInterface dialog, int which)
												{
														Log.i(logTag, String.format("Forcing boat %s to home home", boat.getName()));
														boat.goHome(new ToastFailureCallback("go home message timed out"));
												}
										})
										.setNegativeButton("No", new DialogInterface.OnClickListener()
										{
												public void onClick(DialogInterface dialog, int which) { }
										})
										.show();
				}
				else
				{
						Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
				}
		}

		public void sendPID()
		{
				loadPreferences();
				Boat boat = currentBoat();
				if (boat != null)
				{
						boat.setPID(tPID, rPID, new ToastFailureCallback("Set PID Msg timed out"));
				}
				else
				{
						Toast.makeText(context, "Connect to a boat first", Toast.LENGTH_SHORT).show();
				}
		}

		public void latestWaypointPoll()
		{
				final Handler handler = new Handler();
				handler.post(new Runnable()
				{
						@Override
						public void run()
						{
								for (Map.Entry<String, Boat> entry : boats_map.entrySet())
								{
										String boat_name = entry.getKey();
										Boat boat = entry.getValue();
										Path path = path_map.get(boat_name);
										int cpwi = boat.getWaypointsIndex();
										current_wp_index_map.put(boat_name, cpwi);
										// if the wp index has changed, need to redraw the lines
										if (cpwi != old_wp_index_map.get(boat_name))
										{
												// need to update the line colors
												if (path != null && path.getPoints().size() > 0)
												{
														removeWaypaths(boat_name);
														addWaypaths(boat_name);
												}
										}
										old_wp_index_map.put(boat_name, cpwi);
								}
								handler.postDelayed(this, 3000);
						}
				});
		}

		public void alertsAndAlarms()
		{
				final SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
				final Handler handler = new Handler();
				handler.post(new Runnable()
				{
						@Override
						public void run()
						{
								double batt_volt;
								synchronized (_batteryVoltageLock)
								{
										batt_volt = battery_voltage;
								}
								Log.i(logTag, "checking battery voltage...");
								String sleep_str = sharedPref.getString(SettingsActivity.KEY_PREF_SNOOZE, "1");
								long sleep_ms = (int) Double.parseDouble(sleep_str) * 60 * 1000;
								long current_time = System.currentTimeMillis();
								if (current_time - sleep_start_time >= sleep_ms)
								{
										sleep_start_time = 0;
										double alert_voltage = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_VOLTAGE_ALERT, "15.0"));
										double alarm_voltage = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_VOLTAGE_ALARM, "14.0"));
										if (alarm_voltage > alert_voltage) alarm_voltage = (alert_voltage - 0.5);
										String message = String.format("Boat battery = %.2fV", batt_volt);
										Log.i(logTag, message);
										if (batt_volt == 0.0)
										{
												// initial value before connecting to boat
												handler.postDelayed(this, 10000);
												return;
										}
										if (batt_volt < alert_voltage)
										{
												NotificationCompat.Builder mBuilder = new NotificationCompat.Builder(getApplicationContext())
																.setSmallIcon(R.drawable.logo) //just some random icon placeholder
																.setContentTitle("Boat battery warning")
																.setContentText(message);
												if (batt_volt < alarm_voltage)
												{
														if (!alarm_on)
														{
																alarm_ringtone.play();
																alarm_on = true;
														}
														Toast.makeText(getApplicationContext(), String.format("%s, retrieve the boat ASAP!", message), Toast.LENGTH_LONG).show();
												}
												if (!alarm_on)
												{
														mBuilder.setSound(soundUri); // Sound only if there isn't an alarm going
												}
												notificationManager.notify(0, mBuilder.build());
										}
										else
										{
												if (alarm_ringtone.isPlaying()) alarm_ringtone.stop();
												alarm_on = false;
										}
								}
								else
								{
										Log.i(logTag, "battery alerts snoozing...");
								}

								// run at least once every 60 seconds
								long sleep_remaining = Math.max(100, Math.min(current_time - sleep_start_time, 60000));
								handler.postDelayed(this, sleep_remaining);
						}
				});
		}

		void set_speed_spinner_from_pref()
		{
				final SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
				String vehicle_speed = sharedPref.getString(SettingsActivity.KEY_PREF_SPEED, "MEDIUM");
				if (speed_spinner != null)
				{
						switch (vehicle_speed)
						{
								case "SLOW":
										speed_spinner.setSelection(0);
										break;
								case "MEDIUM":
										speed_spinner.setSelection(1);
										break;
								case "FAST":
										speed_spinner.setSelection(2);
										break;
								case "CUSTOM":
										speed_spinner.setSelection(3);
										break;
								default:
										break;
						}
				}
		}

		private void removeWaypaths(String boat_name)
		{
				synchronized (_wpGraphicsLock)
				{
						ArrayList<Polyline> outline;
						ArrayList<Polyline> top;
						if (!boat_name.isEmpty())
						{
								Log.d(logTag, String.format("removeWaypaths() for %s", boat_name));
								outline = waypath_outline_map.get(boat_name);
								top = waypath_top_map.get(boat_name);
						}
						else
						{
								Log.d(logTag, "removeWaypaths() for unowned path");
								outline = outline_list;
								top = topline_list;
						}

						if (outline != null && outline.size() > 0)
						{
								Log.d(logTag, String.format("outline.size() = %d", outline.size()));
								for (Polyline p : outline)
								{
										mMapboxMap.removeAnnotation(p);
										p.remove();
								}
						}
						if (top != null && top.size() > 0)
						{
								for (Polyline p : top)
								{
										mMapboxMap.removeAnnotation(p);
										p.remove();
								}
						}
				}
		}

		private void addWaypaths(String boat_name)
		{
				synchronized (_wpGraphicsLock)
				{

						ArrayList<Polyline> outline;
						ArrayList<Polyline> top;
						int color;
						Path path;
						int wp_index;
						boolean owned = false;
						if (!boat_name.isEmpty())
						{
								Log.d(logTag, String.format("addWaypaths() for %s", boat_name));
								outline = waypath_outline_map.get(boat_name);
								top = waypath_top_map.get(boat_name);
								path = path_map.get(boat_name);
								wp_index = current_wp_index_map.get(boat_name);
								color = boats_map.get(boat_name).getLineColor();
								owned = true;
						}
						else
						{
								Log.d(logTag, "addWaypaths() for unowned path");
								outline = outline_list;
								top = topline_list;
								path = unowned_path;
								wp_index = -1;
								color = Color.WHITE;
						}

						ArrayList<ArrayList<LatLng>> point_pairs;
						ArrayList<LatLng> pair;
						if (path == null) return;
						point_pairs = path.getPointPairs();
						for (int i = 0; i < point_pairs.size(); i++)
						{
								pair = point_pairs.get(i);
								outline.add(mMapboxMap.addPolyline(new PolylineOptions().addAll(pair).color(Color.BLACK).width(8)));
								// i = 0 is waypoints (0, 1) --> should be white until current wp index = 2
								// i = 1 is waypoints (1, 2) --> should be white until current wp index = 3
								// ...
								if (wp_index > i + 1 || (owned && wp_index == -1))
								{
										top.add(mMapboxMap.addPolyline(new PolylineOptions().addAll(pair).color(Color.GRAY).width(4)));
										Log.v(logTag, String.format("line i = %d, current_waypoint = %d, GRAY", i, wp_index));
								}
								else
								{
										top.add(mMapboxMap.addPolyline(new PolylineOptions().addAll(pair).color(color).width(4)));
										Log.v(logTag, String.format("line i = %d, current_waypoint = %d, COLORED", i, wp_index));
								}
						}
				}
		}

		public void saveSession() throws IOException
		{
				// TODO: what is this even for?
				/* ASDF
				final File sessionFile = new File(getFilesDir() + "/session.txt");
				System.out.println(sessionFile.getAbsolutePath());
				BufferedWriter writer = new BufferedWriter(new FileWriter(sessionFile, false));
				String tempaddr = currentBoat.getIpAddress().toString();
				tempaddr = tempaddr.substring(1, tempaddr.indexOf(":"));
				writer.write(tempaddr);
				writer.write("\n");
				LatLng cameraPan = mMapboxMap.getCameraPosition().target;
				writer.write(cameraPan.getLatitude() + "\n" + cameraPan.getLongitude());
				System.out.println(mMapboxMap.getCameraPosition().target.toString());
				writer.close();
				*/
		}

		public void loadPreferences()
		{
				final SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);

				String iP = sharedPref.getString(SettingsActivity.KEY_PREF_IP, "192.168.1.1");
				String port = sharedPref.getString(SettingsActivity.KEY_PREF_PORT, "11411");

				textIpAddress = iP;
				textIpAddress = textIpAddress.replace("/", ""); // network on main thread error if this doesn't happen
				boatPort = port;

				// get vehicle type and speed preferences
				String vehicle_type = sharedPref.getString(SettingsActivity.KEY_PREF_VEHICLE_TYPE, "PROP");
				String vehicle_speed = sharedPref.getString(SettingsActivity.KEY_PREF_SPEED, "MEDIUM");
				set_speed_spinner_from_pref();

				Log.i(logTag, String.format("Vehicle type = %s, Speed = %s", vehicle_type, vehicle_speed));
				switch (vehicle_type)
				{
						case "PROP":
								switch (vehicle_speed)
								{
										case "SLOW":
												tPID[0] = 0.07;
												tPID[1] = 0.0;
												tPID[2] = 0.0;
												rPID[0] = 0.45;
												rPID[1] = 0;
												rPID[2] = 0.45;
												break;

										case "MEDIUM":
												tPID[0] = 0.2;
												tPID[1] = 0.0;
												tPID[2] = 0.0;
												rPID[0] = 0.8;
												rPID[1] = 0.0;
												rPID[2] = 0.8;
												break;

										case "FAST":
												tPID[0] = 0.6;
												tPID[1] = 0.0;
												tPID[2] = 0.0;
												rPID[0] = 0.7;
												rPID[1] = 0;
												rPID[2] = 0.9;
												break;

										case "CUSTOM":
												tPID[0] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_THRUST_P, "0.2"));
												tPID[1] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_THRUST_I, "0"));
												tPID[2] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_THRUST_D, "0"));
												rPID[0] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_RUDDER_P, "1.0"));
												rPID[1] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_RUDDER_I, "0"));
												rPID[2] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_RUDDER_D, "0.2"));
												break;

										default:
												break;
								}
								break;

						case "AIR":
								switch (vehicle_speed)
								{
										case "SLOW":
												tPID[0] = 0.2;
												tPID[1] = 0.0;
												tPID[2] = 0.0;
												rPID[0] = 0.9;
												rPID[1] = 0.0;
												rPID[2] = 0.9;
												break;

										case "MEDIUM":
												tPID[0] = 0.5;
												tPID[1] = 0.0;
												tPID[2] = 0.0;
												rPID[0] = 0.75;
												rPID[1] = 0.0;
												rPID[2] = 0.9;
												break;

										case "FAST":
												tPID[0] = 0.8;
												tPID[1] = 0;
												tPID[2] = 0;
												rPID[0] = 0.7;
												rPID[1] = 0;
												rPID[2] = 0.9;
												break;

										case "CUSTOM":
												tPID[0] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_THRUST_P, "0.4"));
												tPID[1] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_THRUST_I, "0"));
												tPID[2] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_THRUST_D, "0"));
												rPID[0] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_RUDDER_P, "0.75"));
												rPID[1] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_RUDDER_I, "0"));
												rPID[2] = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_PID_RUDDER_D, "0.90"));
												break;

										default:
												break;
								}
								break;

						default:
								break;
				}

				THRUST_MIN = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_THRUST_MIN, "-1.0"));
				THRUST_MAX = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_THRUST_MAX, "0.3"));

				RUDDER_MIN = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_RUDDER_MIN, "-1.0"));
				RUDDER_MAX = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_RUDDER_MAX, "1.0"));

				Double initialPanLat = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_LAT, "0"));
				Double initialPanLon = Double.parseDouble(sharedPref.getString(SettingsActivity.KEY_PREF_LON, "0"));
				initialPan = new LatLng(initialPanLat, initialPanLon);
				setInitialPan = sharedPref.getBoolean(SettingsActivity.KEY_PREF_SAVE_MAP, true);
		}

		public void calculatePathDistance()
		{
				// calculate the total path length (including current boat position to first waypoint)
				// display it in the GUI
				ArrayList<LatLng> points = (ArrayList<LatLng>)unowned_path.getPoints().clone();
				if (points.size() < 1)
				{
						path_length_value.setText("0");
						return;
				}

				/*
				// include distance from boat current location to first waypoint
				Boat boat = currentBoat();
				if (boat != null && boat.getLocation() != null) points.add(0, boat.getLocation());
				*/

				double total_distance = 0;
				for (int i = 1; i < points.size(); i++)
				{
						total_distance += points.get(i).distanceTo(points.get(i-1));
				}
				path_length_value.setText(Long.toString(Math.round(total_distance)));
		}
}
