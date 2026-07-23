import React from "react";

const join = (...classes) => classes.filter(Boolean).join(" ");

export const DashboardCard = ({
  as: Component = "section",
  className = "",
  interactive = false,
  children,
  ...props
}) => (
  <Component
    className={join(
      "dashboard-card",
      interactive && "dashboard-card--interactive",
      className,
    )}
    {...props}
  >
    {children}
  </Component>
);

export const SectionHeader = ({
  eyebrow,
  title,
  description,
  action,
  className = "",
  as: Heading = "h1",
}) => (
  <div className={join("section-header", className)}>
    <div className="min-w-0">
      {eyebrow ? <p className="section-header__eyebrow">{eyebrow}</p> : null}
      <Heading className="section-header__title">{title}</Heading>
      {description ? (
        <p className="section-header__description">{description}</p>
      ) : null}
    </div>
    {action ? <div className="section-header__action">{action}</div> : null}
  </div>
);

const statusTone = {
  connected: "success",
  online: "success",
  active: "success",
  success: "success",
  disconnected: "danger",
  error: "danger",
  failed: "danger",
  warning: "warning",
  idle: "neutral",
  unknown: "neutral",
  info: "info",
};

export const StatusBadge = ({
  status = "unknown",
  label,
  pulse = false,
  className = "",
}) => {
  const tone = statusTone[status] || status;
  return (
    <span className={join("status-badge", `status-badge--${tone}`, className)}>
      <span className="status-badge__indicator" aria-hidden="true">
        {pulse ? <span className="status-badge__pulse" /> : null}
        <span className="status-badge__dot" />
      </span>
      <span>{label || status}</span>
    </span>
  );
};

export const IconButton = ({ label, className = "", children, ...props }) => (
  <button
    type="button"
    aria-label={label}
    title={label}
    className={join("icon-button", className)}
    {...props}
  >
    {children}
  </button>
);

export const MetricCard = ({
  label,
  value,
  unit,
  meta,
  icon,
  className = "",
  children,
}) => (
  <DashboardCard className={join("metric-card", className)}>
    <div className="metric-card__topline">
      <p className="metric-card__label">{label}</p>
      {icon ? <span className="metric-card__icon">{icon}</span> : null}
    </div>
    {value !== undefined ? (
      <p className="metric-card__value">
        {value}
        {unit ? <span className="metric-card__unit">{unit}</span> : null}
      </p>
    ) : null}
    {meta ? <div className="metric-card__meta">{meta}</div> : null}
    {children}
  </DashboardCard>
);

export const ChartCard = ({
  title,
  value,
  status,
  children,
  className = "",
}) => (
  <DashboardCard className={join("chart-card", className)}>
    <div className="chart-card__header">
      <div>
        <p className="chart-card__label">{title}</p>
        {value ? <p className="chart-card__value">{value}</p> : null}
      </div>
      {status}
    </div>
    {children}
  </DashboardCard>
);

export const EmptyState = ({
  title,
  description,
  action,
  icon,
  className = "",
}) => (
  <div className={join("empty-state", className)}>
    {icon ? <div className="empty-state__icon">{icon}</div> : null}
    <p className="empty-state__title">{title}</p>
    {description ? (
      <p className="empty-state__description">{description}</p>
    ) : null}
    {action ? <div className="empty-state__action">{action}</div> : null}
  </div>
);

export const LoadingSkeleton = ({ className = "", lines = 3 }) => (
  <div
    className={join("loading-skeleton", className)}
    aria-label="Loading"
    role="status"
  >
    {Array.from({ length: lines }, (_, index) => (
      <span
        key={index}
        className="loading-skeleton__line"
        style={{ width: `${Math.max(46, 100 - index * 17)}%` }}
      />
    ))}
  </div>
);
