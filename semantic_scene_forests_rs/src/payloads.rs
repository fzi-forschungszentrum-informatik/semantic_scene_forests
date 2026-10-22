// Copyright (c) 2026 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

use crate::{ffi::bridge, Detection3D, Image, ObjectNode};
use ros_env::{geometry_msgs::msg, vision_msgs::msg as vision};

impl From<&msg::Pose> for bridge::PoseRecord {
    fn from(pose: &msg::Pose) -> Self {
        Self {
            position: [pose.position.x, pose.position.y, pose.position.z],
            orientation: [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        }
    }
}

impl From<bridge::PoseRecord> for msg::Pose {
    fn from(pose: bridge::PoseRecord) -> Self {
        Self {
            position: msg::Point {
                x: pose.position[0],
                y: pose.position[1],
                z: pose.position[2],
            },
            orientation: msg::Quaternion {
                x: pose.orientation[0],
                y: pose.orientation[1],
                z: pose.orientation[2],
                w: pose.orientation[3],
            },
        }
    }
}

impl From<&Detection3D> for bridge::DetectionRecord {
    fn from(detection: &Detection3D) -> Self {
        Self {
            header: (&detection.header).into(),
            results: detection
                .results
                .iter()
                .map(|result| bridge::HypothesisRecord {
                    class_id: result.hypothesis.class_id.clone(),
                    score: result.hypothesis.score,
                    pose: (&result.pose.pose).into(),
                    covariance: result.pose.covariance,
                })
                .collect(),
            center: (&detection.bbox.center).into(),
            size: [
                detection.bbox.size.x,
                detection.bbox.size.y,
                detection.bbox.size.z,
            ],
            id: detection.id.clone(),
        }
    }
}

impl From<bridge::DetectionRecord> for Detection3D {
    fn from(detection: bridge::DetectionRecord) -> Self {
        Self {
            header: detection.header.into(),
            results: detection
                .results
                .into_iter()
                .map(|result| vision::ObjectHypothesisWithPose {
                    hypothesis: vision::ObjectHypothesis {
                        class_id: result.class_id,
                        score: result.score,
                    },
                    pose: msg::PoseWithCovariance {
                        pose: result.pose.into(),
                        covariance: result.covariance,
                    },
                })
                .collect(),
            bbox: vision::BoundingBox3D {
                center: detection.center.into(),
                size: msg::Vector3 {
                    x: detection.size[0],
                    y: detection.size[1],
                    z: detection.size[2],
                },
            },
            id: detection.id,
        }
    }
}

impl From<&Image> for bridge::ImageRecord {
    fn from(image: &Image) -> Self {
        Self {
            header: (&image.header).into(),
            height: image.height,
            width: image.width,
            encoding: image.encoding.clone(),
            is_bigendian: image.is_bigendian,
            step: image.step,
            data: image.data.clone(),
        }
    }
}

impl From<bridge::ImageRecord> for Image {
    fn from(image: bridge::ImageRecord) -> Self {
        Self {
            header: image.header.into(),
            height: image.height,
            width: image.width,
            encoding: image.encoding,
            is_bigendian: image.is_bigendian,
            step: image.step,
            data: image.data,
        }
    }
}

impl bridge::PayloadRecord {
    pub(crate) fn new(detection: Option<&Detection3D>, visuals: [Option<&Image>; 5]) -> Self {
        Self {
            has_detection: detection.is_some(),
            detection: detection.map(Into::into).unwrap_or_default(),
            visuals: visuals.map(|image| bridge::ImageSlot {
                present: image.is_some(),
                image: image.map(Into::into).unwrap_or_default(),
            }),
        }
    }
}

impl From<bridge::ObjectNodeRecord> for ObjectNode {
    fn from(node: bridge::ObjectNodeRecord) -> Self {
        Self {
            id: node.id,
            class_name: node.class_name,
            detection: node
                .payload
                .has_detection
                .then(|| node.payload.detection.into()),
            visuals: node
                .payload
                .visuals
                .map(|slot| slot.present.then(|| slot.image.into())),
        }
    }
}
