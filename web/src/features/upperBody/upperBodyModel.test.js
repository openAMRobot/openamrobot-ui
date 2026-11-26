import {
  CONFIGURATION_UNKNOWN,
  describeConfiguration,
  findPlaceholderLinks,
  isCombinedDescription,
  linkHeightAbove,
  liftReadout,
} from "./upperBodyModel";
import { COMBINED_URDF, PLATFORM_ONLY_URDF } from "./__fixtures__/combinedUrdf";

const REPORTED = {
  column: "dold_v4_350",
  base_plate: "bp000",
  head_pitch: "hp30",
  base_tilt: "bt10",
  datums_revision: "2026-10-28.a",
};

describe("describeConfiguration", () => {
  it("is unknown when nothing is reported", () => {
    expect(describeConfiguration(null)).toMatchObject({ known: false, message: CONFIGURATION_UNKNOWN });
  });

  it("is unknown when any field is missing, and fills nothing in", () => {
    const { datums_revision, ...partial } = REPORTED;
    const result = describeConfiguration(partial);
    expect(result.known).toBe(false);
    expect(result.fields).toEqual([]);
  });

  it("shows the reported values unchanged", () => {
    const result = describeConfiguration(REPORTED);
    expect(result.known).toBe(true);
    expect(result.fields.map((f) => f.value)).toEqual(["dold_v4_350", "bp000", "hp30", "bt10", "2026-10-28.a"]);
    expect(result.flags).toEqual([]);
  });

  it("flags a robot that reports hp25 and shows it as reported", () => {
    const result = describeConfiguration({ ...REPORTED, head_pitch: "hp25" });
    expect(result.fields.find((f) => f.key === "head_pitch").value).toBe("hp25");
    expect(result.flags).toHaveLength(1);
    expect(result.flags[0]).toMatch(/hp25/);
    expect(result.flags[0]).toMatch(/hp30/);
  });
});

describe("model parsing", () => {
  it("recognises the combined model only", () => {
    expect(isCombinedDescription(COMBINED_URDF)).toBe(true);
    expect(isCombinedDescription(PLATFORM_ONLY_URDF)).toBe(false);
    expect(isCombinedDescription(null)).toBe(false);
    expect(isCombinedDescription("<not xml")).toBe(false);
  });

  it("lists links that carry placeholder geometry", () => {
    expect(findPlaceholderLinks(COMBINED_URDF)).toEqual(["lift_column_base_link", "lift_carriage_link"]);
  });

  it("computes the shoulder height from the reported lift position", () => {
    expect(linkHeightAbove(COMBINED_URDF, "base_footprint", "openarm_left_base_link", { lift_joint: 0 })).toBeCloseTo(1.0, 6);
    expect(linkHeightAbove(COMBINED_URDF, "base_footprint", "openarm_left_base_link", { lift_joint: 0.175 })).toBeCloseTo(1.175, 6);
  });

  it("returns null rather than assume an unreported lift position", () => {
    expect(linkHeightAbove(COMBINED_URDF, "base_footprint", "openarm_left_base_link", {})).toBeNull();
    expect(linkHeightAbove(COMBINED_URDF, "base_footprint", "missing_link", { lift_joint: 0 })).toBeNull();
  });
});

describe("liftReadout", () => {
  it("places the shoulder height against L1000, L1175 and L1350", () => {
    const r = liftReadout({ liftPositionM: 0.17, shoulderM: 1.17 });
    expect(r.nearest.id).toBe("L1175");
    expect(r.nearest.deltaMm).toBeCloseTo(-5, 6);
    expect(r.fraction).toBeCloseTo(170 / 350, 6);
    expect(liftReadout({ liftPositionM: 0.35, shoulderM: 1.35 }).nearest.id).toBe("L1350");
  });

  it("reports nothing when the shoulder height is unknown", () => {
    expect(liftReadout({ liftPositionM: undefined, shoulderM: null })).toEqual({
      liftPositionM: null,
      shoulderMm: null,
      nearest: null,
      fraction: null,
    });
  });
});
