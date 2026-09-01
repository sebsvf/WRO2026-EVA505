def should_enter_obstacle_avoid(ctx) -> bool:
    return (ctx.pillar.pillar_detected
            and ctx.pillar.confidence >= ctx.min_confidence)


def should_exit_obstacle_avoid(ctx) -> bool:
    return not ctx.pillar.pillar_detected


def lane_confidence_lost(ctx) -> bool:
    return ctx.lane.confidence < ctx.min_confidence


def lane_confidence_recovered(ctx) -> bool:
    return ctx.lane.confidence >= ctx.min_confidence


def laps_complete(ctx) -> bool:
    return ctx.completed_laps >= ctx.laps_required


def corner_section_entered(ctx, prev_curvature: float, curvature_threshold=0.35) -> bool:
    """
    Cheap corner-section detector used for lap counting (see Sec 4.9:
    "least mature part of the design"). A corner section is inferred
    when curvature crosses the threshold from below, since the WRO
    field is laid out as 4 corner + 4 straight sections per lap.

    This is intentionally simple and WILL misfire on noisy curvature
    estimates -- the encoder-based cross-check in fsm.py's
    _update_lap_counter() exists specifically to catch that.
    """
    return prev_curvature < curvature_threshold <= abs(ctx.lane.curvature)


def parking_marker_found(ctx) -> bool:
    return (ctx.parking.parking_lot_detected
            and ctx.parking.confidence >= ctx.min_confidence)


def parking_marker_lost_from_fov(ctx) -> bool:
    """
    Used to trigger the switch from vision-guided alignment to the
    blind, encoder-based reverse approach (Sec 4.9 / parking_detection.py).
    """
    return not ctx.parking.parking_lot_detected
