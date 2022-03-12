// store is the place where the shared global state is managed. Modification of
// the state is only allowed via the store method calls, typically happening
// only in a single place. By doing this we maintain a unidirectional data flow
// to simplify the logic.
//

var store = {
  showPlots: false,
  drawing_polygon: false,
  robots: {},
  current_robot_name: '',
  pathNames: [],
  gps_path: [],
  displayed_path: [],
  displayed_path_name: "",
  displayed_dense_path: [],
  path_start: -1,
  path_end: -1,
  path_point_to_remove: -1,
  have_cleared_autonomy: {}, // {robot name => true / false}
  first_path_point: {}
};
