package com.platypus.android.tablet;

import com.platypus.crw.VehicleServer;

public class PointOfInterest
{
    double[] location;
    String desc;
    VehicleServer.MapMarkerTypes type;
    long id;

    PointOfInterest(long _id, double[] _location, VehicleServer.MapMarkerTypes _type, String _desc)
    {
        id = _id;
        location = _location.clone();
        type = _type;
        desc = _desc;
    }
}
