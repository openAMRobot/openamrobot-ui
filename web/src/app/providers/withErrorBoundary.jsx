import React from "react";

import ErrorBoundry from "../ErrorBoundary";

class ErrorBoundaryComponent extends React.Component {
  constructor(props) {
    super(props);
    this.state = { error: null };
  }

  componentDidCatch(error) {
    this.setState({
      error,
    });
  }

  render() {
    if (this.state.error) {
      return <ErrorBoundry />;
    }
    return this.props.children;
  }
}

const withBoundry = (component) => () => (
  <ErrorBoundaryComponent>{component()}</ErrorBoundaryComponent>
);

export default withBoundry;
