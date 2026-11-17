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

use crate::ffi::bridge;
use crate::{Crop, ObjectNode};
use ros_env::{geometry_msgs::msg, vision_msgs::msg::BoundingBox3D};

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

impl From<&BoundingBox3D> for bridge::BBoxRecord {
    fn from(bbox: &BoundingBox3D) -> Self {
        Self {
            center: (&bbox.center).into(),
            size: [bbox.size.x, bbox.size.y, bbox.size.z],
        }
    }
}

impl From<bridge::BBoxRecord> for BoundingBox3D {
    fn from(bbox: bridge::BBoxRecord) -> Self {
        Self {
            center: bbox.center.into(),
            size: msg::Vector3 {
                x: bbox.size[0],
                y: bbox.size[1],
                z: bbox.size[2],
            },
        }
    }
}

impl From<&ObjectNode> for bridge::ObjectNodeRecord {
    fn from(node: &ObjectNode) -> Self {
        Self {
            id: node.id,
            class_name: node.class_name.clone(),
            bbox: (&node.bbox).into(),
            frame_id: node.frame_id.clone(),
            source: node.source,
            last_seen: (&node.last_seen).into(),
            last_updated: (&node.last_updated).into(),
            crops: node
                .crops
                .iter()
                .map(|crop| bridge::CropRecord {
                    stamp: (&crop.stamp).into(),
                    jpeg: crop.jpeg.clone(),
                })
                .collect(),
            workers: node
                .workers
                .iter()
                .map(|(name, processed)| bridge::WorkerRecord {
                    name: name.clone(),
                    processed: *processed,
                })
                .collect(),
        }
    }
}

impl From<bridge::ObjectNodeRecord> for ObjectNode {
    fn from(node: bridge::ObjectNodeRecord) -> Self {
        Self {
            id: node.id,
            class_name: node.class_name,
            bbox: node.bbox.into(),
            frame_id: node.frame_id,
            source: node.source,
            last_seen: node.last_seen.into(),
            last_updated: node.last_updated.into(),
            crops: node
                .crops
                .into_iter()
                .map(|crop| Crop {
                    stamp: crop.stamp.into(),
                    jpeg: crop.jpeg,
                })
                .collect(),
            workers: node
                .workers
                .into_iter()
                .map(|worker| (worker.name, worker.processed))
                .collect(),
        }
    }
}
