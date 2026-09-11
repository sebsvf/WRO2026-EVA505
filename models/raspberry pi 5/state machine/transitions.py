"""Pure transition predicates, kept separate for readable tests."""


def lane_confidence_lost(ctx):
    return ctx.lane.confidence < ctx.min_confidence


def laps_complete(ctx):
    return ctx.completed_laps >= ctx.laps_required


def parking_marker_found(ctx):
    return (
        ctx.parking.parking_lot_detected
        and ctx.parking.marker_count >= 2
        and ctx.parking.confidence >= ctx.min_confidence
    )
