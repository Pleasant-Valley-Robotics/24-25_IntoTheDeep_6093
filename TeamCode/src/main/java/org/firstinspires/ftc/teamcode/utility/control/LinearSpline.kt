package org.firstinspires.ftc.teamcode.utility.control

import kotlin.math.sqrt

private typealias Segment = Pair<PoseData, PoseData>

class LinearSpline(
    val speed: Double,
    vararg targetPoints: PoseData,
) : Trajectory {
    val targetSegments = targetPoints.zip(targetPoints.drop(1))

    companion object {
        private fun closestPoint(segment: Segment, target: PoseData): Double {
            val (x1, y1) = segment.first.pos
            val (x2, y2) = segment.second.pos
            val (x3, y3) = target.pos

            val u = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) /
                    (segment.second - segment.first).pos.let { it * it }

            return u.coerceIn(0.0..1.0)
        }

        private fun intersect(
            segment: Segment,
            target: PoseData,
            distance: Double
        ): Pair<Double?, Double?> {
            // https://stackoverflow.com/a/1084899
            val diff = segment.second.pos - segment.first.pos
            val f = segment.first.pos - target.pos
            val a = diff * diff
            val b = f * diff * 2.0
            val c = f * f - distance * distance
            val ds = b * b - 4.0 * a * c
            if (ds < 0.0) return Pair(null, null)

            val t1 = (-b - sqrt(ds)) / (2.0 * a)
            val t2 = (-b + sqrt(ds)) / (2.0 * a)

            return Pair(
                t1.takeIf { it in 0.0..1.0 },
                t2.takeIf { it in 0.0..1.0 },
            )
        }

        private fun lerp(segment: Segment, t: Double) = segment.first * (1 - t) + segment.second * t
    }

    override val start = 0.0
    override val end = targetSegments.size.toDouble()

    override fun nearestPoint(pose: PoseData) =
        targetSegments
            .mapIndexed { i, s -> closestPoint(s, pose) + i.toDouble() }
            .minBy { (this.getPos(it) - pose).pos.length }

    override fun pointsAround(pose: PoseData, dist: Double) = buildList {
        for ((i, segment) in targetSegments.withIndex()) {
            val (fst, snd) = intersect(segment, pose, dist)
            fst?.let { add(it + i.toDouble()) }
            snd?.let { add(it + i.toDouble()) }
        }
    }

    override fun getPos(t: Double): PoseData {
        val target = t.toInt()
        val inter = t % 1.0
        if (target == targetSegments.size && inter == 0.0) return targetSegments.last().second

        return lerp(targetSegments[target], inter)
    }

    override fun getVel(t: Double): PoseData {
        val target = t.toInt().let { if (it == targetSegments.size) it - 1 else it }
        val segment = targetSegments[target]
        val vec = (segment.second - segment.first).pos

        return PoseData(vec / vec.length * speed, 0.0)
    }
}