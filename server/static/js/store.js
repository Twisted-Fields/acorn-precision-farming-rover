// store is the place where the shared global state is managed. Modification of
// the state is only allowed via the store method calls, typically happening
// only in a single place. By doing this we maintain a unidirectional data flow
// to simplify the logic.
//

var store = {
  debug: true,
  robotData: {},
};
