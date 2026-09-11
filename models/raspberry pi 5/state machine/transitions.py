def should_enter_obstacle_avoid(ctx) -> bool:
    return (
        ctx.pillar.pillar_detected
        and ctx.pillar.confidence >= ctx.min_confidence
    )


def should_exit_obstacle_avoid(ctx) -> bool:
    return (
        not ctx.pillar.pillar_detected
        or ctx.pillar.confidence < ctx.min_confidence
    )


def lane_confidence_lost(ctx) -> bool:
    return ctx.lane.confidence < ctx.min_confidence


def lane_confidence_recovered(ctx) -> bool:
    return ctx.lane.confidence >= ctx.min_confidence


def laps_complete(ctx) -> bool:
    return ctx.completed_laps >= ctx.laps_required


def corner_section_entered(ctx, prev_curvature: float,
                           curvature_threshold: float = 0.35) -> bool:
    current_curvature = abs(ctx.lane.curvature)

    return (
        prev_curvature < curvature_threshold
        and current_curvature >= curvature_threshold
    )


def parking_marker_found(ctx) -> bool:
    return (
        ctx.parking.parking_lot_detected
        and ctx.parking.confidence >= ctx.min_confidence
    )


def parking_marker_lost_from_fov(ctx) -> bool:
    return (
        not ctx.parking.parking_lot_detected
        and ctx.parking.confidence < 0.2
    )