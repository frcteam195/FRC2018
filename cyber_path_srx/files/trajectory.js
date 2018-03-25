/* Generated from Java with JSweet 2.0.0 - http://www.jsweet.org */
/**
 * Do cubic spline interpolation between points.
 *
 * @author Art Kalb
 * @author Jared341
 * @class
 */
var Spline = (function () {
    function Spline() {
        this.type_ = null;
        this.a_ = 0;
        this.b_ = 0;
        this.c_ = 0;
        this.d_ = 0;
        this.e_ = 0;
        this.y_offset_ = 0;
        this.x_offset_ = 0;
        this.knot_distance_ = 0;
        this.theta_offset_ = 0;
        this.arc_length_ = 0;
        this.arc_length_ = -1;
    }
    Spline.CubicHermite_$LI$ = function () { if (Spline.CubicHermite == null)
        Spline.CubicHermite = new Spline.Type("CubicHermite"); return Spline.CubicHermite; };
    ;
    Spline.QuinticHermite_$LI$ = function () { if (Spline.QuinticHermite == null)
        Spline.QuinticHermite = new Spline.Type("QuinticHermite"); return Spline.QuinticHermite; };
    ;
    Spline.almostEqual = function (x, y) {
        return Math.abs(x - y) < 1.0E-6;
    };
    Spline.reticulateSplines$WaypointSequence_Waypoint$WaypointSequence_Waypoint$Spline$Spline_Type = function (start, goal, result, type) {
        return Spline.reticulateSplines$double$double$double$double$double$double$Spline$Spline_Type(start.x, start.y, start.theta, goal.x, goal.y, goal.theta, result, type);
    };
    Spline.reticulateSplines$double$double$double$double$double$double$Spline$Spline_Type = function (x0, y0, theta0, x1, y1, theta1, result, type) {
        console.info("Reticulating splines...");
        result.type_ = type;
        result.x_offset_ = x0;
        result.y_offset_ = y0;
        var x1_hat = Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        if (x1_hat === 0) {
            return false;
        }
        result.knot_distance_ = x1_hat;
        result.theta_offset_ = Math.atan2(y1 - y0, x1 - x0);
        var theta0_hat = ChezyMath.getDifferenceInAngleRadians(result.theta_offset_, theta0);
        var theta1_hat = ChezyMath.getDifferenceInAngleRadians(result.theta_offset_, theta1);
        if (Spline.almostEqual(Math.abs(theta0_hat), Math.PI / 2) || Spline.almostEqual(Math.abs(theta1_hat), Math.PI / 2)) {
            return false;
        }
        if (Math.abs(ChezyMath.getDifferenceInAngleRadians(theta0_hat, theta1_hat)) >= Math.PI / 2) {
            return false;
        }
        var yp0_hat = Math.tan(theta0_hat);
        var yp1_hat = Math.tan(theta1_hat);
        if (type === Spline.CubicHermite_$LI$()) {
            result.a_ = 0;
            result.b_ = 0;
            result.c_ = (yp1_hat + yp0_hat) / (x1_hat * x1_hat);
            result.d_ = -(2 * yp0_hat + yp1_hat) / x1_hat;
            result.e_ = yp0_hat;
        }
        else if (type === Spline.QuinticHermite_$LI$()) {
            result.a_ = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat);
            result.b_ = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat);
            result.c_ = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat);
            result.d_ = 0;
            result.e_ = yp0_hat;
        }
        return true;
    };
    Spline.reticulateSplines = function (x0, y0, theta0, x1, y1, theta1, result, type) {
        if (((typeof x0 === 'number') || x0 === null) && ((typeof y0 === 'number') || y0 === null) && ((typeof theta0 === 'number') || theta0 === null) && ((typeof x1 === 'number') || x1 === null) && ((typeof y1 === 'number') || y1 === null) && ((typeof theta1 === 'number') || theta1 === null) && ((result != null && result instanceof Spline) || result === null) && ((type != null && type instanceof Spline.Type) || type === null)) {
            return Spline.reticulateSplines$double$double$double$double$double$double$Spline$Spline_Type(x0, y0, theta0, x1, y1, theta1, result, type);
        }
        else if (((x0 != null && x0 instanceof WaypointSequence.Waypoint) || x0 === null) && ((y0 != null && y0 instanceof WaypointSequence.Waypoint) || y0 === null) && ((theta0 != null && theta0 instanceof Spline) || theta0 === null) && ((x1 != null && x1 instanceof Spline.Type) || x1 === null) && y1 === undefined && theta1 === undefined && result === undefined && type === undefined) {
            return Spline.reticulateSplines$WaypointSequence_Waypoint$WaypointSequence_Waypoint$Spline$Spline_Type(x0, y0, theta0, x1);
        }
        else
            throw new Error('invalid overload');
    };
    Spline.prototype.calculateLength = function () {
        if (this.arc_length_ >= 0) {
            return this.arc_length_;
        }
        var kNumSamples = 100000;
        var arc_length = 0;
        var t;
        var dydt;
        var integrand;
        var last_integrand = Math.sqrt(1 + this.derivativeAt(0) * this.derivativeAt(0)) / kNumSamples;
        for (var i = 1; i <= kNumSamples; ++i) {
            t = i / kNumSamples;
            dydt = this.derivativeAt(t);
            integrand = Math.sqrt(1 + dydt * dydt) / kNumSamples;
            arc_length += (integrand + last_integrand) / 2;
            last_integrand = integrand;
        }
        ;
        this.arc_length_ = this.knot_distance_ * arc_length;
        return this.arc_length_;
    };
    Spline.prototype.getPercentageForDistance = function (distance) {
        var kNumSamples = 100000;
        var arc_length = 0;
        var t = 0;
        var last_arc_length = 0;
        var dydt;
        var integrand;
        var last_integrand = Math.sqrt(1 + this.derivativeAt(0) * this.derivativeAt(0)) / kNumSamples;
        distance /= this.knot_distance_;
        for (var i = 1; i <= kNumSamples; ++i) {
            t = i / kNumSamples;
            dydt = this.derivativeAt(t);
            integrand = Math.sqrt(1 + dydt * dydt) / kNumSamples;
            arc_length += (integrand + last_integrand) / 2;
            if (arc_length > distance) {
                break;
            }
            last_integrand = integrand;
            last_arc_length = arc_length;
        }
        ;
        var interpolated = t;
        if (arc_length !== last_arc_length) {
            interpolated += ((distance - last_arc_length) / (arc_length - last_arc_length) - 1) / kNumSamples;
        }
        return interpolated;
    };
    Spline.prototype.getXandY = function (percentage) {
        var result = [0, 0];
        percentage = Math.max(Math.min(percentage, 1), 0);
        var x_hat = percentage * this.knot_distance_;
        var y_hat = (this.a_ * x_hat + this.b_) * x_hat * x_hat * x_hat * x_hat + this.c_ * x_hat * x_hat * x_hat + this.d_ * x_hat * x_hat + this.e_ * x_hat;
        var cos_theta = Math.cos(this.theta_offset_);
        var sin_theta = Math.sin(this.theta_offset_);
        result[0] = x_hat * cos_theta - y_hat * sin_theta + this.x_offset_;
        result[1] = x_hat * sin_theta + y_hat * cos_theta + this.y_offset_;
        return result;
    };
    Spline.prototype.valueAt = function (percentage) {
        percentage = Math.max(Math.min(percentage, 1), 0);
        var x_hat = percentage * this.knot_distance_;
        var y_hat = (this.a_ * x_hat + this.b_) * x_hat * x_hat * x_hat * x_hat + this.c_ * x_hat * x_hat * x_hat + this.d_ * x_hat * x_hat + this.e_ * x_hat;
        var cos_theta = Math.cos(this.theta_offset_);
        var sin_theta = Math.sin(this.theta_offset_);
        var value = x_hat * sin_theta + y_hat * cos_theta + this.y_offset_;
        return value;
    };
    Spline.prototype.derivativeAt = function (percentage) {
        percentage = Math.max(Math.min(percentage, 1), 0);
        var x_hat = percentage * this.knot_distance_;
        var yp_hat = (5 * this.a_ * x_hat + 4 * this.b_) * x_hat * x_hat * x_hat + 3 * this.c_ * x_hat * x_hat + 2 * this.d_ * x_hat + this.e_;
        return yp_hat;
    };
    Spline.prototype.secondDerivativeAt = function (percentage) {
        percentage = Math.max(Math.min(percentage, 1), 0);
        var x_hat = percentage * this.knot_distance_;
        var ypp_hat = (20 * this.a_ * x_hat + 12 * this.b_) * x_hat * x_hat + 6 * this.c_ * x_hat + 2 * this.d_;
        return ypp_hat;
    };
    Spline.prototype.angleAt = function (percentage) {
        var angle = ChezyMath.boundAngle0to2PiRadians(Math.atan(this.derivativeAt(percentage)) + this.theta_offset_);
        return angle;
    };
    Spline.prototype.angleChangeAt = function (percentage) {
        return ChezyMath.boundAngleNegPiToPiRadians(Math.atan(this.secondDerivativeAt(percentage)));
    };
    Spline.prototype.toString = function () {
        return "a=" + this.a_ + "; b=" + this.b_ + "; c=" + this.c_ + "; d=" + this.d_ + "; e=" + this.e_;
    };
    return Spline;
}());
Spline["__class"] = "Spline";
(function (Spline) {
    var Type = (function () {
        function Type(value) {
            this.value_ = null;
            this.value_ = value;
        }
        Type.prototype.toString = function () {
            return this.value_;
        };
        return Type;
    }());
    Spline.Type = Type;
    Type["__class"] = "Spline.Type";
})(Spline || (Spline = {}));
/**
 * This class holds a bunch of static methods and variables needed for
 * mathematics
 * @class
 */
var ChezyMath = (function () {
    function ChezyMath() {
    }
    ChezyMath.nan_$LI$ = function () { if (ChezyMath.nan == null)
        ChezyMath.nan = (0.0 / 0.0); return ChezyMath.nan; };
    ;
    /*private*/ ChezyMath.mxatan = function (arg) {
        var argsq;
        var value;
        argsq = arg * arg;
        value = ((((ChezyMath.p4 * argsq + ChezyMath.p3) * argsq + ChezyMath.p2) * argsq + ChezyMath.p1) * argsq + ChezyMath.p0);
        value = value / (((((argsq + ChezyMath.q4) * argsq + ChezyMath.q3) * argsq + ChezyMath.q2) * argsq + ChezyMath.q1) * argsq + ChezyMath.q0);
        return value * arg;
    };
    /*private*/ ChezyMath.msatan = function (arg) {
        if (arg < ChezyMath.sq2m1) {
            return ChezyMath.mxatan(arg);
        }
        if (arg > ChezyMath.sq2p1) {
            return ChezyMath.PIO2 - ChezyMath.mxatan(1 / arg);
        }
        return ChezyMath.PIO2 / 2 + ChezyMath.mxatan((arg - 1) / (arg + 1));
    };
    ChezyMath.atan = function (arg) {
        if (arg > 0) {
            return ChezyMath.msatan(arg);
        }
        return -ChezyMath.msatan(-arg);
    };
    ChezyMath.atan2 = function (arg1, arg2) {
        if (arg1 + arg2 === arg1) {
            if (arg1 >= 0) {
                return ChezyMath.PIO2;
            }
            return -ChezyMath.PIO2;
        }
        arg1 = ChezyMath.atan(arg1 / arg2);
        if (arg2 < 0) {
            if (arg1 <= 0) {
                return arg1 + Math.PI;
            }
            return arg1 - Math.PI;
        }
        return arg1;
    };
    ChezyMath.asin = function (arg) {
        var temp;
        var sign;
        sign = 0;
        if (arg < 0) {
            arg = -arg;
            sign++;
        }
        if (arg > 1) {
            return ChezyMath.nan_$LI$();
        }
        temp = Math.sqrt(1 - arg * arg);
        if (arg > 0.7) {
            temp = ChezyMath.PIO2 - ChezyMath.atan(temp / arg);
        }
        else {
            temp = ChezyMath.atan(arg / temp);
        }
        if (sign > 0) {
            temp = -temp;
        }
        return temp;
    };
    ChezyMath.acos = function (arg) {
        if (arg > 1 || arg < -1) {
            return ChezyMath.nan_$LI$();
        }
        return ChezyMath.PIO2 - ChezyMath.asin(arg);
    };
    /**
     * Get the difference in angle between two angles.
     *
     * @param {number} from The first angle
     * @param {number} to The second angle
     * @return {number} The change in angle from the first argument necessary to line up
     * with the second. Always between -Pi and Pi
     */
    ChezyMath.getDifferenceInAngleRadians = function (from, to) {
        return ChezyMath.boundAngleNegPiToPiRadians(to - from);
    };
    /**
     * Get the difference in angle between two angles.
     *
     * @param {number} from The first angle
     * @param {number} to The second angle
     * @return {number} The change in angle from the first argument necessary to line up
     * with the second. Always between -180 and 180
     */
    ChezyMath.getDifferenceInAngleDegrees = function (from, to) {
        return ChezyMath.boundAngleNeg180to180Degrees(to - from);
    };
    ChezyMath.boundAngle0to360Degrees = function (angle) {
        while ((angle >= 360.0)) {
            angle -= 360.0;
        }
        ;
        while ((angle < 0.0)) {
            angle += 360.0;
        }
        ;
        return angle;
    };
    ChezyMath.boundAngleNeg180to180Degrees = function (angle) {
        while ((angle >= 180.0)) {
            angle -= 360.0;
        }
        ;
        while ((angle < -180.0)) {
            angle += 360.0;
        }
        ;
        return angle;
    };
    ChezyMath.boundAngle0to2PiRadians = function (angle) {
        while ((angle >= 2.0 * Math.PI)) {
            angle -= 2.0 * Math.PI;
        }
        ;
        while ((angle < 0.0)) {
            angle += 2.0 * Math.PI;
        }
        ;
        return angle;
    };
    ChezyMath.boundAngleNegPiToPiRadians = function (angle) {
        while ((angle >= Math.PI)) {
            angle -= 2.0 * Math.PI;
        }
        ;
        while ((angle < -Math.PI)) {
            angle += 2.0 * Math.PI;
        }
        ;
        return angle;
    };
    return ChezyMath;
}());
ChezyMath.sq2p1 = 2.414213562373095;
ChezyMath.sq2m1 = 0.41421356237309503;
ChezyMath.p4 = 16.15364129822302;
ChezyMath.p3 = 268.42548195503974;
ChezyMath.p2 = 1153.029351540485;
ChezyMath.p1 = 1780.406316433197;
ChezyMath.p0 = 896.7859740366387;
ChezyMath.q4 = 58.95697050844462;
ChezyMath.q3 = 536.2653740312153;
ChezyMath.q2 = 1666.7838148816338;
ChezyMath.q1 = 2079.33497444541;
ChezyMath.q0 = 896.7859740366387;
ChezyMath.PIO2 = 1.5707963267948966;
ChezyMath["__class"] = "ChezyMath";
/**
 * Factory class for creating Trajectories.
 *
 * @author Jared341
 * @class
 */
var TrajectoryGenerator = (function () {
    function TrajectoryGenerator() {
    }
    TrajectoryGenerator.StepStrategy_$LI$ = function () { if (TrajectoryGenerator.StepStrategy == null)
        TrajectoryGenerator.StepStrategy = new TrajectoryGenerator.Strategy("StepStrategy"); return TrajectoryGenerator.StepStrategy; };
    ;
    TrajectoryGenerator.TrapezoidalStrategy_$LI$ = function () { if (TrajectoryGenerator.TrapezoidalStrategy == null)
        TrajectoryGenerator.TrapezoidalStrategy = new TrajectoryGenerator.Strategy("TrapezoidalStrategy"); return TrajectoryGenerator.TrapezoidalStrategy; };
    ;
    TrajectoryGenerator.SCurvesStrategy_$LI$ = function () { if (TrajectoryGenerator.SCurvesStrategy == null)
        TrajectoryGenerator.SCurvesStrategy = new TrajectoryGenerator.Strategy("SCurvesStrategy"); return TrajectoryGenerator.SCurvesStrategy; };
    ;
    TrajectoryGenerator.AutomaticStrategy_$LI$ = function () { if (TrajectoryGenerator.AutomaticStrategy == null)
        TrajectoryGenerator.AutomaticStrategy = new TrajectoryGenerator.Strategy("AutomaticStrategy"); return TrajectoryGenerator.AutomaticStrategy; };
    ;
    TrajectoryGenerator.RectangularIntegration_$LI$ = function () { if (TrajectoryGenerator.RectangularIntegration == null)
        TrajectoryGenerator.RectangularIntegration = new TrajectoryGenerator.IntegrationMethod("RectangularIntegration"); return TrajectoryGenerator.RectangularIntegration; };
    ;
    TrajectoryGenerator.TrapezoidalIntegration_$LI$ = function () { if (TrajectoryGenerator.TrapezoidalIntegration == null)
        TrajectoryGenerator.TrapezoidalIntegration = new TrajectoryGenerator.IntegrationMethod("TrapezoidalIntegration"); return TrajectoryGenerator.TrapezoidalIntegration; };
    ;
    /**
     * Generate a trajectory from a start state to a goal state.
     *
     * Read the notes on each of the Strategies defined above, as certain
     * arguments are ignored for some strategies.
     *
     * @param {TrajectoryGenerator.Config} config Definition of constraints and sampling rate (WARNING: Some
     * may be ignored)
     * @param {TrajectoryGenerator.Strategy} strategy Which generator to use
     * @param {number} start_vel The starting velocity (WARNING: May be ignored)
     * @param {number} start_heading The starting heading
     * @param {number} goal_pos The goal position
     * @param {number} goal_vel The goal velocity (WARNING: May be ignored)
     * @param {number} goal_heading The goal heading
     * @return {Trajectory} A Trajectory that satisfies the relevant constraints and end
     * conditions.
     */
    TrajectoryGenerator.generate = function (config, strategy, start_vel, start_heading, goal_pos, goal_vel, goal_heading) {
        if (strategy === TrajectoryGenerator.AutomaticStrategy_$LI$()) {
            strategy = TrajectoryGenerator.chooseStrategy(start_vel, goal_vel, config.max_vel);
        }
        var traj;
        if (strategy === TrajectoryGenerator.StepStrategy_$LI$()) {
            var impulse = (goal_pos / config.max_vel) / config.dt;
            var time = ((Math.floor(impulse)) | 0);
            traj = TrajectoryGenerator.secondOrderFilter(1, 1, config.dt, config.max_vel, config.max_vel, impulse, time, TrajectoryGenerator.TrapezoidalIntegration_$LI$());
        }
        else if (strategy === TrajectoryGenerator.TrapezoidalStrategy_$LI$()) {
            var start_discount = 0.5 * start_vel * start_vel / config.max_acc;
            var end_discount = 0.5 * goal_vel * goal_vel / config.max_acc;
            var adjusted_max_vel = Math.min(config.max_vel, Math.sqrt(config.max_acc * goal_pos - start_discount - end_discount));
            var t_rampup = (adjusted_max_vel - start_vel) / config.max_acc;
            var x_rampup = start_vel * t_rampup + 0.5 * config.max_acc * t_rampup * t_rampup;
            var t_rampdown = (adjusted_max_vel - goal_vel) / config.max_acc;
            var x_rampdown = adjusted_max_vel * t_rampdown - 0.5 * config.max_acc * t_rampdown * t_rampdown;
            var x_cruise = goal_pos - x_rampdown - x_rampup;
            var time = (((t_rampup + t_rampdown + x_cruise / adjusted_max_vel) / config.dt + 0.5) | 0);
            var f1_length = (Math.ceil((adjusted_max_vel / config.max_acc) / config.dt) | 0);
            var impulse = (goal_pos / adjusted_max_vel) / config.dt - start_vel / config.max_acc / config.dt + start_discount + end_discount;
            traj = TrajectoryGenerator.secondOrderFilter(f1_length, 1, config.dt, start_vel, adjusted_max_vel, impulse, time, TrajectoryGenerator.TrapezoidalIntegration_$LI$());
        }
        else if (strategy === TrajectoryGenerator.SCurvesStrategy_$LI$()) {
            var adjusted_max_vel = Math.min(config.max_vel, (-config.max_acc * config.max_acc + Math.sqrt(config.max_acc * config.max_acc * config.max_acc * config.max_acc + 4 * config.max_jerk * config.max_jerk * config.max_acc * goal_pos)) / (2 * config.max_jerk));
            var f1_length = (Math.ceil((adjusted_max_vel / config.max_acc) / config.dt) | 0);
            var f2_length = (Math.ceil((config.max_acc / config.max_jerk) / config.dt) | 0);
            var impulse = (goal_pos / adjusted_max_vel) / config.dt;
            var time = ((Math.ceil(f1_length + f2_length + impulse)) | 0);
            traj = TrajectoryGenerator.secondOrderFilter(f1_length, f2_length, config.dt, 0, adjusted_max_vel, impulse, time, TrajectoryGenerator.TrapezoidalIntegration_$LI$());
        }
        else {
            return null;
        }
        var total_heading_change = goal_heading - start_heading;
        for (var i = 0; i < traj.getNumSegments(); ++i) {
            traj.segments_[i].heading = start_heading + total_heading_change * (traj.segments_[i].pos) / traj.segments_[traj.getNumSegments() - 1].pos;
        }
        ;
        return traj;
    };
    TrajectoryGenerator.secondOrderFilter = function (f1_length, f2_length, dt, start_vel, max_vel, total_impulse, length, integration) {
        if (length <= 0) {
            return null;
        }
        var traj = new Trajectory(length);
        var last = new Trajectory.Segment();
        last.pos = 0;
        last.vel = start_vel;
        last.acc = 0;
        last.jerk = 0;
        last.dt = dt;
        var f1 = (function (s) { var a = []; while (s-- > 0)
            a.push(0); return a; })(length);
        f1[0] = (start_vel / max_vel) * f1_length;
        var f2;
        for (var i = 0; i < length; ++i) {
            var input = Math.min(total_impulse, 1);
            if (input < 1) {
                input -= 1;
                total_impulse = 0;
            }
            else {
                total_impulse -= input;
            }
            var f1_last = void 0;
            if (i > 0) {
                f1_last = f1[i - 1];
            }
            else {
                f1_last = f1[0];
            }
            f1[i] = Math.max(0.0, Math.min(f1_length, f1_last + input));
            f2 = 0;
            for (var j = 0; j < f2_length; ++j) {
                if (i - j < 0) {
                    break;
                }
                f2 += f1[i - j];
            }
            ;
            f2 = f2 / f1_length;
            traj.segments_[i].vel = f2 / f2_length * max_vel;
            if (integration === TrajectoryGenerator.RectangularIntegration_$LI$()) {
                traj.segments_[i].pos = traj.segments_[i].vel * dt + last.pos;
            }
            else if (integration === TrajectoryGenerator.TrapezoidalIntegration_$LI$()) {
                traj.segments_[i].pos = (last.vel + traj.segments_[i].vel) / 2.0 * dt + last.pos;
            }
            traj.segments_[i].x = traj.segments_[i].pos;
            traj.segments_[i].y = 0;
            traj.segments_[i].acc = (traj.segments_[i].vel - last.vel) / dt;
            traj.segments_[i].jerk = (traj.segments_[i].acc - last.acc) / dt;
            traj.segments_[i].dt = dt;
            last = traj.segments_[i];
        }
        ;
        return traj;
    };
    TrajectoryGenerator.chooseStrategy = function (start_vel, goal_vel, max_vel) {
        var strategy;
        if (start_vel === goal_vel && start_vel === max_vel) {
            strategy = TrajectoryGenerator.StepStrategy_$LI$();
        }
        else if (start_vel === goal_vel && start_vel === 0) {
            strategy = TrajectoryGenerator.SCurvesStrategy_$LI$();
        }
        else {
            strategy = TrajectoryGenerator.TrapezoidalStrategy_$LI$();
        }
        return strategy;
    };
    return TrajectoryGenerator;
}());
TrajectoryGenerator["__class"] = "TrajectoryGenerator";
(function (TrajectoryGenerator) {
    var Config = (function () {
        function Config() {
            this.dt = 0;
            this.max_vel = 0;
            this.max_acc = 0;
            this.max_jerk = 0;
        }
        return Config;
    }());
    TrajectoryGenerator.Config = Config;
    Config["__class"] = "TrajectoryGenerator.Config";
    var Strategy = (function () {
        function Strategy(value) {
            this.value_ = null;
            this.value_ = value;
        }
        Strategy.prototype.toString = function () {
            return this.value_;
        };
        return Strategy;
    }());
    TrajectoryGenerator.Strategy = Strategy;
    Strategy["__class"] = "TrajectoryGenerator.Strategy";
    var IntegrationMethod = (function () {
        function IntegrationMethod(value) {
            this.value_ = null;
            this.value_ = value;
        }
        IntegrationMethod.prototype.toString = function () {
            return this.value_;
        };
        return IntegrationMethod;
    }());
    TrajectoryGenerator.IntegrationMethod = IntegrationMethod;
    IntegrationMethod["__class"] = "TrajectoryGenerator.IntegrationMethod";
})(TrajectoryGenerator || (TrajectoryGenerator = {}));
/**
 * Implementation of a Trajectory using arrays as the underlying storage
 * mechanism.
 *
 * @author Jared341
 * @param {number} length
 * @class
 */
var Trajectory = (function () {
    function Trajectory(segments) {
        var _this = this;
        this.segments_ = null;
        this.inverted_y_ = false;
        if (((segments != null && segments instanceof Array && (segments.length == 0 || segments[0] == null || (segments[0] != null && segments[0] instanceof Trajectory.Segment))) || segments === null)) {
            var __args = Array.prototype.slice.call(arguments);
            this.segments_ = null;
            this.inverted_y_ = false;
            (function () {
                _this.segments_ = segments;
            })();
        }
        else if (((typeof segments === 'number') || segments === null)) {
            var __args = Array.prototype.slice.call(arguments);
            var length_1 = __args[0];
            this.segments_ = null;
            this.inverted_y_ = false;
            (function () {
                _this.segments_ = new Array(length_1);
                for (var i = 0; i < length_1; ++i) {
                    _this.segments_[i] = new Trajectory.Segment();
                }
                ;
            })();
        }
        else
            throw new Error('invalid overload');
    }
    Trajectory.prototype.setInvertedY = function (inverted) {
        this.inverted_y_ = inverted;
    };
    Trajectory.prototype.getNumSegments = function () {
        return this.segments_.length;
    };
    Trajectory.prototype.getSegment = function (index) {
        if (index < this.getNumSegments()) {
            if (!this.inverted_y_) {
                return this.segments_[index];
            }
            else {
                var segment = new Trajectory.Segment(this.segments_[index]);
                segment.y *= -1.0;
                segment.heading *= -1.0;
                return segment;
            }
        }
        else {
            return new Trajectory.Segment();
        }
    };
    Trajectory.prototype.setSegment = function (index, segment) {
        if (index < this.getNumSegments()) {
            this.segments_[index] = segment;
        }
    };
    Trajectory.prototype.scale = function (scaling_factor) {
        for (var i = 0; i < this.getNumSegments(); ++i) {
            this.segments_[i].pos *= scaling_factor;
            this.segments_[i].vel *= scaling_factor;
            this.segments_[i].acc *= scaling_factor;
            this.segments_[i].jerk *= scaling_factor;
        }
        ;
    };
    Trajectory.prototype.append = function (to_append) {
        var temp = new Array(this.getNumSegments() + to_append.getNumSegments());
        for (var i = 0; i < this.getNumSegments(); ++i) {
            temp[i] = new Trajectory.Segment(this.segments_[i]);
        }
        ;
        for (var i = 0; i < to_append.getNumSegments(); ++i) {
            temp[i + this.getNumSegments()] = new Trajectory.Segment(to_append.getSegment(i));
        }
        ;
        this.segments_ = temp;
    };
    Trajectory.prototype.copy = function () {
        var cloned = new Trajectory(this.getNumSegments());
        cloned.segments_ = this.copySegments(this.segments_);
        return cloned;
    };
    Trajectory.prototype.reverse = function () {
        var cloned = new Trajectory(1);
        cloned.segments_ = this.reverseSegments(this.segments_);
        return cloned;
    };
    Trajectory.prototype.reverseSegments = function (tocopy) {
        var count = tocopy.length;
        var start_pos = tocopy[count - 1].pos;
        var copied = new Array(count);
        for (var i = 0; i < tocopy.length; ++i) {
            copied[i] = new Trajectory.Segment(tocopy[count - i - 1]);
            copied[i].reverse();
            copied[i].pos -= start_pos;
        }
        ;
        return copied;
    };
    Trajectory.prototype.copySegments = function (tocopy) {
        var copied = new Array(tocopy.length);
        for (var i = 0; i < tocopy.length; ++i) {
            copied[i] = new Trajectory.Segment(tocopy[i]);
        }
        ;
        return copied;
    };
    Trajectory.prototype.toString = function () {
        var str = "Segment\tPos\tVel\tAcc\tJerk\tHeading\n";
        for (var i = 0; i < this.getNumSegments(); ++i) {
            var segment = this.getSegment(i);
            str += i + "\t";
            str += segment.pos + "\t";
            str += segment.vel + "\t";
            str += segment.acc + "\t";
            str += segment.jerk + "\t";
            str += segment.heading + "\t";
            str += "\n";
        }
        ;
        return str;
    };
    Trajectory.prototype.toStringProfile = function () {
        return this.toString();
    };
    Trajectory.prototype.toStringEuclidean = function () {
        var str = "Segment\tx\ty\tHeading\n";
        for (var i = 0; i < this.getNumSegments(); ++i) {
            var segment = this.getSegment(i);
            str += i + "\t";
            str += segment.x + "\t";
            str += segment.y + "\t";
            str += segment.heading + "\t";
            str += "\n";
        }
        ;
        return str;
    };
    return Trajectory;
}());
Trajectory["__class"] = "Trajectory";
(function (Trajectory) {
    var Pair = (function () {
        function Pair(left, right, center) {
            this.left = null;
            this.right = null;
            this.center = null;
            this.left = left;
            this.right = right;
            this.center = center;
        }
        Pair.prototype.reverse = function () {
            var result = new Trajectory.Pair(this.left.reverse(), this.right.reverse());
            return result;
        };
        return Pair;
    }());
    Trajectory.Pair = Pair;
    Pair["__class"] = "Trajectory.Pair";
    var Segment = (function () {
        function Segment(pos, vel, acc, jerk, heading, dt, x, y) {
            var _this = this;
            if (((typeof pos === 'number') || pos === null) && ((typeof vel === 'number') || vel === null) && ((typeof acc === 'number') || acc === null) && ((typeof jerk === 'number') || jerk === null) && ((typeof heading === 'number') || heading === null) && ((typeof dt === 'number') || dt === null) && ((typeof x === 'number') || x === null) && ((typeof y === 'number') || y === null)) {
                var __args = Array.prototype.slice.call(arguments);
                this.pos = 0;
                this.vel = 0;
                this.acc = 0;
                this.jerk = 0;
                this.heading = 0;
                this.dt = 0;
                this.x = 0;
                this.y = 0;
                this.pos = 0;
                this.vel = 0;
                this.acc = 0;
                this.jerk = 0;
                this.heading = 0;
                this.dt = 0;
                this.x = 0;
                this.y = 0;
                (function () {
                    _this.pos = pos;
                    _this.vel = vel;
                    _this.acc = acc;
                    _this.jerk = jerk;
                    _this.heading = heading;
                    _this.dt = dt;
                    _this.x = x;
                    _this.y = y;
                })();
            }
            else if (((pos != null && pos instanceof Trajectory.Segment) || pos === null) && vel === undefined && acc === undefined && jerk === undefined && heading === undefined && dt === undefined && x === undefined && y === undefined) {
                var __args = Array.prototype.slice.call(arguments);
                var to_copy_1 = __args[0];
                this.pos = 0;
                this.vel = 0;
                this.acc = 0;
                this.jerk = 0;
                this.heading = 0;
                this.dt = 0;
                this.x = 0;
                this.y = 0;
                this.pos = 0;
                this.vel = 0;
                this.acc = 0;
                this.jerk = 0;
                this.heading = 0;
                this.dt = 0;
                this.x = 0;
                this.y = 0;
                (function () {
                    _this.pos = to_copy_1.pos;
                    _this.vel = to_copy_1.vel;
                    _this.acc = to_copy_1.acc;
                    _this.jerk = to_copy_1.jerk;
                    _this.heading = to_copy_1.heading;
                    _this.dt = to_copy_1.dt;
                    _this.x = to_copy_1.x;
                    _this.y = to_copy_1.y;
                })();
            }
            else if (pos === undefined && vel === undefined && acc === undefined && jerk === undefined && heading === undefined && dt === undefined && x === undefined && y === undefined) {
                var __args = Array.prototype.slice.call(arguments);
                this.pos = 0;
                this.vel = 0;
                this.acc = 0;
                this.jerk = 0;
                this.heading = 0;
                this.dt = 0;
                this.x = 0;
                this.y = 0;
                this.pos = 0;
                this.vel = 0;
                this.acc = 0;
                this.jerk = 0;
                this.heading = 0;
                this.dt = 0;
                this.x = 0;
                this.y = 0;
            }
            else
                throw new Error('invalid overload');
        }
        Segment.prototype.reverse = function () {
            this.vel = -this.vel;
            this.acc = -this.acc;
            this.jerk = -this.jerk;
        };
        Segment.prototype.toString = function () {
            return "pos: " + this.pos + "; vel: " + this.vel + "; acc: " + this.acc + "; jerk: " + this.jerk + "; heading: " + this.heading;
        };
        return Segment;
    }());
    Trajectory.Segment = Segment;
    Segment["__class"] = "Trajectory.Segment";
})(Trajectory || (Trajectory = {}));
/**
 * Base class for an autonomous path.
 *
 * @author Jared341
 * @param {string} name
 * @param {Trajectory.Pair} go_left_pair
 * @class
 */
var Path = (function () {
    function Path(name, go_left_pair) {
        var _this = this;
        if (((typeof name === 'string') || name === null) && ((go_left_pair != null && go_left_pair instanceof Trajectory.Pair) || go_left_pair === null)) {
            var __args = Array.prototype.slice.call(arguments);
            this.go_left_pair_ = null;
            this.name_ = null;
            this.go_left_ = false;
            this.go_left_pair_ = null;
            this.name_ = null;
            this.go_left_ = false;
            (function () {
                _this.name_ = name;
                _this.go_left_pair_ = go_left_pair;
                _this.go_left_ = true;
            })();
        }
        else if (name === undefined && go_left_pair === undefined) {
            var __args = Array.prototype.slice.call(arguments);
            this.go_left_pair_ = null;
            this.name_ = null;
            this.go_left_ = false;
            this.go_left_pair_ = null;
            this.name_ = null;
            this.go_left_ = false;
        }
        else
            throw new Error('invalid overload');
    }
    Path.prototype.getName = function () {
        return this.name_;
    };
    Path.prototype.goLeft = function () {
        this.go_left_ = true;
        this.go_left_pair_.left.setInvertedY(false);
        this.go_left_pair_.right.setInvertedY(false);
    };
    Path.prototype.goRight = function () {
        this.go_left_ = false;
        this.go_left_pair_.left.setInvertedY(true);
        this.go_left_pair_.right.setInvertedY(true);
    };
    Path.prototype.getLeftWheelTrajectory = function () {
        return (this.go_left_ ? this.go_left_pair_.left : this.go_left_pair_.right);
    };
    Path.prototype.getCenterTrajectory = function () {
        return this.go_left_pair_.center;
    };
    Path.prototype.getRightWheelTrajectory = function () {
        return (this.go_left_ ? this.go_left_pair_.right : this.go_left_pair_.left);
    };
    Path.prototype.getPair = function () {
        return this.go_left_pair_;
    };
    Path.prototype.getEndHeading = function () {
        var numSegments = this.getLeftWheelTrajectory().getNumSegments();
        var lastSegment = this.getLeftWheelTrajectory().getSegment(numSegments - 1);
        return lastSegment.heading;
    };
    Path.prototype.reverse = function () {
        this.go_left_pair_ = this.go_left_pair_.reverse();
    };
    return Path;
}());
Path["__class"] = "Path";
/**
 * A WaypointSequence is a sequence of Waypoints.  #whatdidyouexpect
 *
 * @author Art Kalb
 * @author Stephen Pinkerton
 * @author Jared341
 * @param {number} max_size
 * @class
 */
var WaypointSequence = (function () {
    function WaypointSequence(max_size) {
        this.waypoints_ = null;
        this.num_waypoints_ = 0;
        this.waypoints_ = new Array(max_size);
    }
    WaypointSequence.prototype.addWaypoint = function (w) {
        if (this.num_waypoints_ < this.waypoints_.length) {
            this.waypoints_[this.num_waypoints_] = w;
            ++this.num_waypoints_;
        }
    };
    WaypointSequence.prototype.getNumWaypoints = function () {
        return this.num_waypoints_;
    };
    WaypointSequence.prototype.getWaypoint = function (index) {
        if (index >= 0 && index < this.getNumWaypoints()) {
            return this.waypoints_[index];
        }
        else {
            return null;
        }
    };
    WaypointSequence.prototype.invertY = function () {
        var inverted = new WaypointSequence(this.waypoints_.length);
        inverted.num_waypoints_ = this.num_waypoints_;
        for (var i = 0; i < this.num_waypoints_; ++i) {
            inverted.waypoints_[i] = this.waypoints_[i];
            inverted.waypoints_[i].y *= -1;
            inverted.waypoints_[i].theta = ChezyMath.boundAngle0to2PiRadians(2 * Math.PI - inverted.waypoints_[i].theta);
        }
        ;
        return inverted;
    };
    return WaypointSequence;
}());
WaypointSequence["__class"] = "WaypointSequence";
(function (WaypointSequence) {
    var Waypoint = (function () {
        function Waypoint(x, y, theta) {
            var _this = this;
            if (((typeof x === 'number') || x === null) && ((typeof y === 'number') || y === null) && ((typeof theta === 'number') || theta === null)) {
                var __args = Array.prototype.slice.call(arguments);
                this.x = 0;
                this.y = 0;
                this.theta = 0;
                this.x = 0;
                this.y = 0;
                this.theta = 0;
                (function () {
                    _this.x = x;
                    _this.y = y;
                    _this.theta = theta;
                })();
            }
            else if (((x != null && x instanceof WaypointSequence.Waypoint) || x === null) && y === undefined && theta === undefined) {
                var __args = Array.prototype.slice.call(arguments);
                var tocopy_1 = __args[0];
                this.x = 0;
                this.y = 0;
                this.theta = 0;
                this.x = 0;
                this.y = 0;
                this.theta = 0;
                (function () {
                    _this.x = tocopy_1.x;
                    _this.y = tocopy_1.y;
                    _this.theta = tocopy_1.theta;
                })();
            }
            else
                throw new Error('invalid overload');
        }
        return Waypoint;
    }());
    WaypointSequence.Waypoint = Waypoint;
    Waypoint["__class"] = "WaypointSequence.Waypoint";
})(WaypointSequence || (WaypointSequence = {}));
var PathGenerator = (function () {
    function PathGenerator() {
    }
    /**
     * Generate a path for autonomous driving.
     *
     * @param {WaypointSequence} waypoints The waypoints to drive to (FOR THE "GO LEFT" CASE!!!!)
     * @param {TrajectoryGenerator.Config} config Trajectory config.
     * @param {number} wheelbase_width Wheelbase separation; units must be consistent with
     * config and waypoints.
     * @param {string} name The name of the new path.  THIS MUST BE A VALID JAVA CLASS NAME
     * @return {Path} The path.
     */
    PathGenerator.makePath = function (waypoints, config, wheelbase_width, name) {
        return new Path(name, PathGenerator.generateLeftAndRightFromSeq(waypoints, config, wheelbase_width));
    };
    PathGenerator.generateLeftAndRightFromSeq = function (path, config, wheelbase_width) {
        return PathGenerator.makeLeftAndRightTrajectories(PathGenerator.generateFromPath(path, config), wheelbase_width);
    };
    PathGenerator.generateFromPath = function (path, config) {
        if (path.getNumWaypoints() < 2) {
            return null;
        }
        var splines = new Array(path.getNumWaypoints() - 1);
        var spline_lengths = (function (s) { var a = []; while (s-- > 0)
            a.push(0); return a; })(splines.length);
        var total_distance = 0;
        for (var i = 0; i < splines.length; ++i) {
            splines[i] = new Spline();
            if (!Spline.reticulateSplines$WaypointSequence_Waypoint$WaypointSequence_Waypoint$Spline$Spline_Type(path.getWaypoint(i), path.getWaypoint(i + 1), splines[i], Spline.QuinticHermite_$LI$())) {
                return null;
            }
            spline_lengths[i] = splines[i].calculateLength();
            total_distance += spline_lengths[i];
        }
        ;
        var traj = TrajectoryGenerator.generate(config, TrajectoryGenerator.SCurvesStrategy_$LI$(), 0.0, path.getWaypoint(0).theta, total_distance, 0.0, path.getWaypoint(0).theta);
        var cur_spline = 0;
        var cur_spline_start_pos = 0;
        var length_of_splines_finished = 0;
        for (var i = 0; i < traj.getNumSegments(); ++i) {
            var cur_pos = traj.getSegment(i).pos;
            var found_spline = false;
            while ((!found_spline)) {
                var cur_pos_relative = cur_pos - cur_spline_start_pos;
                if (cur_pos_relative <= spline_lengths[cur_spline]) {
                    var percentage = splines[cur_spline].getPercentageForDistance(cur_pos_relative);
                    traj.getSegment(i).heading = splines[cur_spline].angleAt(percentage);
                    var coords = splines[cur_spline].getXandY(percentage);
                    traj.getSegment(i).x = coords[0];
                    traj.getSegment(i).y = coords[1];
                    found_spline = true;
                }
                else if (cur_spline < splines.length - 1) {
                    length_of_splines_finished += spline_lengths[cur_spline];
                    cur_spline_start_pos = length_of_splines_finished;
                    ++cur_spline;
                }
                else {
                    traj.getSegment(i).heading = splines[splines.length - 1].angleAt(1.0);
                    var coords = splines[splines.length - 1].getXandY(1.0);
                    traj.getSegment(i).x = coords[0];
                    traj.getSegment(i).y = coords[1];
                    found_spline = true;
                }
            }
            ;
        }
        ;
        return traj;
    };
    /**
     * Generate left and right wheel trajectories from a reference.
     *
     * @param {Trajectory} input The reference trajectory.
     * @param {number} wheelbase_width The center-to-center distance between the left and
     * right sides.
     * @return {Trajectory.Pair} [0] is left, [1] is right
     */
    PathGenerator.makeLeftAndRightTrajectories = function (input, wheelbase_width) {
        var output = new Array(2);
        output[0] = input.copy();
        output[1] = input.copy();
        var left = output[0];
        var right = output[1];
        for (var i = 0; i < input.getNumSegments(); ++i) {
            var current = input.getSegment(i);
            var cos_angle = Math.cos(current.heading);
            var sin_angle = Math.sin(current.heading);
            var s_left = left.getSegment(i);
            s_left.x = current.x - wheelbase_width / 2 * sin_angle;
            s_left.y = current.y + wheelbase_width / 2 * cos_angle;
            if (i > 0) {
                var dist = Math.sqrt((s_left.x - left.getSegment(i - 1).x) * (s_left.x - left.getSegment(i - 1).x) + (s_left.y - left.getSegment(i - 1).y) * (s_left.y - left.getSegment(i - 1).y));
                s_left.pos = left.getSegment(i - 1).pos + dist;
                s_left.vel = dist / s_left.dt;
                s_left.acc = (s_left.vel - left.getSegment(i - 1).vel) / s_left.dt;
                s_left.jerk = (s_left.acc - left.getSegment(i - 1).acc) / s_left.dt;
            }
            var s_right = right.getSegment(i);
            s_right.x = current.x + wheelbase_width / 2 * sin_angle;
            s_right.y = current.y - wheelbase_width / 2 * cos_angle;
            if (i > 0) {
                var dist = Math.sqrt((s_right.x - right.getSegment(i - 1).x) * (s_right.x - right.getSegment(i - 1).x) + (s_right.y - right.getSegment(i - 1).y) * (s_right.y - right.getSegment(i - 1).y));
                s_right.pos = right.getSegment(i - 1).pos + dist;
                s_right.vel = dist / s_right.dt;
                s_right.acc = (s_right.vel - right.getSegment(i - 1).vel) / s_right.dt;
                s_right.jerk = (s_right.acc - right.getSegment(i - 1).acc) / s_right.dt;
            }
        }
        ;
        return new Trajectory.Pair(output[0], output[1], input.copy());
    };
    return PathGenerator;
}());
PathGenerator["__class"] = "PathGenerator";
TrajectoryGenerator.TrapezoidalIntegration_$LI$();
TrajectoryGenerator.RectangularIntegration_$LI$();
TrajectoryGenerator.AutomaticStrategy_$LI$();
TrajectoryGenerator.SCurvesStrategy_$LI$();
TrajectoryGenerator.TrapezoidalStrategy_$LI$();
TrajectoryGenerator.StepStrategy_$LI$();
ChezyMath.nan_$LI$();
Spline.QuinticHermite_$LI$();
Spline.CubicHermite_$LI$();
