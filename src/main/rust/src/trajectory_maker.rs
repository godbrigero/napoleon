use j4rs::errors::J4RsError;
use j4rs::{Instance, InvocationArg, JavaClass, Jvm, JvmBuilder};
use jni::objects::{JClass, JString};
use jni::objects::{JObject, JValue};
use jni::sys::jdoubleArray;
use jni::JNIEnv;
use nalgebra::{Matrix3, Point2, Rotation2, Transform2, Translation2};

pub struct Pose2d {
    translation: Translation2<f64>,
    rotation: Rotation2<f64>,
}

impl Pose2d {
    pub fn new(translation: Translation2<f64>, rotation: Rotation2<f64>) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub fn to_basic_types(&self) -> Vec<f64> {
        vec![
            self.translation.x,
            self.translation.y,
            self.rotation.matrix().m11,
            self.rotation.matrix().m12,
            self.rotation.matrix().m21,
            self.rotation.matrix().m22,
        ]
    }
}

pub struct TrajectoryMaker {
    instance: Instance,
    transformations: Vec<Pose2d>,
    jvm: Jvm,
}

impl TrajectoryMaker {
    pub fn new(jvm: Jvm, transformations: Vec<Pose2d>) -> Self {
        /*
            list:
            [
                x,
                y,
                rotation matrix (m11, m12, m21, m22)
            ]
        */

        let basic_types = transformations
            .iter()
            .map(|pose| pose.to_basic_types())
            .collect::<Vec<Vec<f64>>>()
            .concat();

        let instance = jvm
            .create_instance(
                "TrajectoryMaker",
                &[InvocationArg::from(
                    to_java_list(jvm.clone(), basic_types).unwrap(),
                )],
            )
            .unwrap();

        Self {
            instance,
            transformations,
            jvm,
        }
    }

    pub fn predict_next_position_trajectory(&self, current_time_ms: f64) {
        let result_instance = self
            .jvm
            .invoke(
                &self.instance,
                "predictNextPosition",
                &[InvocationArg::try_from(current_time_ms).unwrap()],
            )
            .unwrap();

        // Convert Java double[] to Rust Vec<f64>
        let _rust_vec: Vec<f64> = self.jvm.to_rust(result_instance).unwrap(); // TODO: Handle future output
    }
}

pub fn to_java_list(jvm: Jvm, basic_types: Vec<f64>) -> Result<Instance, J4RsError> {
    jvm.java_list(JavaClass::Double, basic_types)
}
