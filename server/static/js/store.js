// store is the place where the shared global state is managed. Modification of
// the state is only allowed via the store method calls, typically happening
// only in a single place. By doing this we maintain a unidirectional data flow
// to simplify the logic.
//

var store = {
  debug: true,
  map: null,
  showPlots: false,
  robots: [],
  robotMarkerStore: {},
  arrowMarkerStore: {},
  savedPathMarkers: {},
  livePathMarkers: [],
  pathNames: [],
  livePathName: "",
  gpsPathMarkers: [],
  gpsPathLength: 0,
  debugPointMarkers: [],
  debugPointsLength: 0,
  gps_path: [],
  displayed_path: [],
  displayed_path_name: "",
  displayed_dense_path: [],
  path_start: -1,
  path_end: -1,
  path_point_to_remove: -1,
  have_cleared_autonomy: false,
  first_path_point: {},
};
