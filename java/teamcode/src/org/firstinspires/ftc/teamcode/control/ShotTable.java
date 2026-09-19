package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.ShotTableData;

/**
 * range -> (hood position, flywheel RPM, margin), generated offline by
 * tools/shottable.ts and baked in by tools/genconstants.mjs.
 *
 * The solver runs on a laptop; the hub only interpolates. That is what makes the port
 * cheap (PLAN.md section 10.3).
 */
public class ShotTable {
    private final double[] range;
    private final double[] hood;
    private final double[] rpm;
    private final double[] margin;

    public ShotTable(double[] range, double[] hood, double[] rpm, double[] margin) {
        this.range = range; this.hood = hood; this.rpm = rpm; this.margin = margin;
    }

    public boolean isEmpty() { return range.length == 0; }
    public double minRange() { return range.length == 0 ? 0 : range[0]; }
    public double maxRange() { return range.length == 0 ? 0 : range[range.length - 1]; }

    private int index(double r) {
        if (r <= range[0]) return 0;
        for (int i = 1; i < range.length; i++) if (r <= range[i]) return i;
        return range.length - 1;
    }

    private double lerp(double[] a, double r) {
        if (range.length == 0) return 0;
        if (r <= range[0]) return a[0];
        if (r >= range[range.length - 1]) return a[range.length - 1];
        int i = index(r);
        double t = (r - range[i - 1]) / (range[i] - range[i - 1]);
        return a[i - 1] + t * (a[i] - a[i - 1]);
    }

    /**
     * Interpolate any column that shares this table's ranges -- the land-probability inputs
     * live in ShotTableData as parallel arrays rather than being copied into every row.
     */
    public double lerpAt(double[] column, double rangeIn) {
        return column.length == range.length ? lerp(column, rangeIn) : 0;
    }

    public double hoodFor(double rangeIn) { return lerp(hood, rangeIn); }
    public double rpmFor(double rangeIn) { return lerp(rpm, rangeIn); }
    public double marginFor(double rangeIn) { return lerp(margin, rangeIn); }

    /**
     * The range where a shot is most likely to LAND: where DriveToRange wants to be.
     *
     * This was the widest speed-margin band, 30-38 in on the shipped table -- exactly the
     * rows where the fewest balls stay in the CELL. Margin says how much wheel error still
     * threads the mouth; a steep close lob threads with huge margin and bounces back out, and
     * the measured stay rate climbs from 86% at 30-42 in to 95% at 54 and 99% at 78
     * (tools/ceiling.ts, tools/landrate.ts). So the band is every row within five points of
     * the best per-shot ceiling the table carries -- the same product LandProbability scores a
     * shot by -- and the target is the NEAR edge of it plus the driver's tolerance: farther
     * buys nothing and costs flight time and field. Mirrors ShotTable.bestBand() in the
     * TypeScript. A table without the model columns falls back to the margin band.
     */
    public double bestRange() {
        if (range.length == 0) return 0;
        double[] score = new double[range.length];
        boolean modelled = ShotTableData.SPEED_LO.length == range.length
                && ShotTableData.SIGMA_SPEED.length == range.length;
        double best = 0;
        for (int i = 0; i < range.length; i++) {
            if (modelled) {
                double lo = ShotTableData.SPEED_LO[i], hi = ShotTableData.SPEED_HI[i];
                double stay = ShotTableData.P_STAY.length == range.length ? ShotTableData.P_STAY[i] : 1;
                score[i] = LandProbability.pThread(lo, hi, (lo + hi) / 2, ShotTableData.SIGMA_SPEED[i]) * stay;
            } else {
                score[i] = margin[i];
            }
            if (score[i] > best) best = score[i];
        }
        double floor = modelled ? best - 0.05 : best * 0.95;
        int lo = -1;
        int hi = -1;
        for (int i = 0; i < range.length; i++) {
            if (score[i] >= floor) {
                if (lo < 0) lo = i;
                hi = i;
            }
        }
        if (lo < 0) return range[0];
        return modelled ? range[lo] + BEST_RANGE_IN_FROM_EDGE : (range[lo] + range[hi]) / 2;
    }

    /** How far inside the near edge of the best band to aim the ranging, inches. */
    private static final double BEST_RANGE_IN_FROM_EDGE = 6.0;

    /** Is this range worth shooting from at all? */
    public boolean usable(double rangeIn) {
        return !isEmpty() && rangeIn >= minRange() && rangeIn <= maxRange() && marginFor(rangeIn) >= 0.02;
    }
}
