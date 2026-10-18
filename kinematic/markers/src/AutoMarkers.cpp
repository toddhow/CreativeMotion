// Automatic marker placement: docs/tasks/0011-a7-automatic-markers.md.

#include "opas/markers/AutoMarkers.hpp"

#include "opas/markers/Validation.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace opas::markers {
namespace {

// How deep the "nearest surface layer" is in the snap's second pass: points
// up to this much farther along the line than the nearest still count.
constexpr double kSheetM = 0.01;

bool isFinite(math::Vec3 v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

AutoAuthorResult authorUnchecked(const PoseClip& clip, const std::array<BodyMapping, 2>& bodyMaps,
                                 const AutoAuthorSettings& settings)
{
    AutoAuthorResult result;
    // Rule 1: detection with the auto profile (Low is never included).
    DetectionSettings detection = settings.detection;
    detection.includeLowConfidence = false;
    const DetectionResult detected = detectContacts(clip, bodyMaps, detection);
    if (!detected.valid) {
        result.error = "detection: " + detected.error;
        return result;
    }
    const double clipS =
        clip.sampleRateHz > 0.0 ? static_cast<double>(clip.frameCount()) / clip.sampleRateHz : 0.0;

    // Rule 2: steady contacts only; windows from 0 for the whole node.
    MarkerSet candidates;
    std::vector<std::size_t> reportIndex; // candidate -> its report entry
    for (const DetectedContact& c : detected.contacts) {
        AutoContactReport entry;
        entry.id = "auto-" + c.marker.id;
        entry.confidence = c.confidence;
        entry.durationS = c.durationS;
        if (c.durationS < settings.minShareOfClip * clipS) {
            entry.reason = "intermittent: lasts less than the required share of the clip";
        } else {
            InteractionMarker m = c.marker;
            m.id = entry.id;
            m.window = TimeWindow{0.0, settings.windowEndS};
            candidates.markers.push_back(std::move(m));
            reportIndex.push_back(result.report.size());
        }
        result.report.push_back(std::move(entry));
    }

    // Rule 3: each candidate must pass validation at the bodies as recorded.
    // Validation evaluates a marker at round(startS * rate) = frame 0 here.
    if (!candidates.markers.empty()) {
        ValidationSettings asRecorded;
        asRecorded.scales = {1.0};
        const ValidationReport report = validateMarkers(candidates, clip, bodyMaps, asRecorded);
        if (!report.valid) {
            result.error = "validation: " + report.error;
            return result;
        }
        for (std::size_t i = 0; i < candidates.markers.size(); ++i) {
            AutoContactReport& entry = result.report[reportIndex[i]];
            if (report.markers[i].passed()) {
                entry.kept = true;
                result.markers.markers.push_back(candidates.markers[i]);
            } else {
                entry.reason = "not met at the recorded bodies";
            }
        }
    }
    result.valid = true;
    return result;
}

} // namespace

AutoAuthorResult autoAuthorMarkers(const PoseClip& clip, const std::array<BodyMapping, 2>& bodyMaps,
                                   const AutoAuthorSettings& settings)
{
    try {
        return authorUnchecked(clip, bodyMaps, settings);
    } catch (const std::exception& e) {
        // Only allocation can throw here; the contract is "never throws".
        AutoAuthorResult result;
        result.error = std::string("internal error: ") + e.what();
        return result;
    }
}

SnapResult snapOffsetToSurface(math::Vec3 offsetM, math::Vec3 inwardDirection,
                               std::span<const math::Vec3> surfacePointsM,
                               const SnapSettings& settings)
{
    SnapResult result;
    result.offsetM = offsetM;
    const double length = math::length(inwardDirection);
    if (!isFinite(offsetM) || !isFinite(inwardDirection) || !(length > 0.0)) {
        return result;
    }
    const math::Vec3 d = inwardDirection / length;
    // Pass 1: how far along the line the nearest surface lies.
    double nearestAlong = std::numeric_limits<double>::infinity();
    for (const math::Vec3& q : surfacePointsM) {
        if (!isFinite(q)) {
            continue;
        }
        const math::Vec3 w = q - offsetM;
        const double along = math::dot(w, d);
        if (math::length(w - along * d) <= settings.snapRadiusM) {
            nearestAlong = std::min(nearestAlong, std::abs(along));
        }
    }
    if (!std::isfinite(nearestAlong)) {
        return result; // no surface near the line
    }
    // Pass 2: within that surface layer, the point closest to the line, so a
    // point at the edge of the search tube cannot pull the marker sideways or
    // short of a curved surface.
    double bestFromLine = std::numeric_limits<double>::infinity();
    double bestAlong = 0.0;
    for (const math::Vec3& q : surfacePointsM) {
        if (!isFinite(q)) {
            continue;
        }
        const math::Vec3 w = q - offsetM;
        const double along = math::dot(w, d);
        const double fromLine = math::length(w - along * d);
        if (fromLine <= settings.snapRadiusM && std::abs(along) <= nearestAlong + kSheetM &&
            fromLine < bestFromLine) { // strict: ties keep the earlier point
            bestFromLine = fromLine;
            bestAlong = along;
        }
    }
    // Move along the line only: the marker keeps its place across the surface.
    result.offsetM = offsetM + (bestAlong - settings.contactOffsetM) * d;
    result.snapped = true;
    result.movedM = math::distance(result.offsetM, offsetM);
    return result;
}

} // namespace opas::markers
