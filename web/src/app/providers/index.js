import compose from "compose-function";
import withRouter from "./withRouter";
import withBoundary from "./withErrorBoundary";
import withStore from "./withStore";

const withProviders = compose(withBoundary, withRouter, withStore);

export default withProviders;
