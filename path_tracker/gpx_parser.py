import gpxpy
import gpxpy.gpx

# Parsing an existing file:
# -------------------------

gpx_file = open('course-1-final-assign-stanley-controller/ruckweg_waypoints.gpx', 'r')
doc = open('course-1-final-assign-stanley-controller/ruckweg_coordinates.txt', 'w')
            
gpx = gpxpy.parse(gpx_file)
print(gpx.tracks)
for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            print('Point at ({0},{1}) -> {2}'.format(point.latitude, point.longitude, point.elevation))
            doc.write(str(point.longitude) + ", "+str(point.latitude)+"\n")
