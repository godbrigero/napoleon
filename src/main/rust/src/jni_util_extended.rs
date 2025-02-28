use jni::objects::{JFloatArray, JIntArray};
use jni::sys::{jfloat, jint};
use jni::JNIEnv;
use nalgebra::Vector2;

pub fn jfloatarray_to_vec(env: &JNIEnv, array: JFloatArray) -> Vec<f32> {
    let len = env.get_array_length(&array).unwrap() as usize;
    let mut buffer: Vec<jfloat> = vec![0.0; len];
    env.get_float_array_region(array, 0, &mut buffer).unwrap();
    buffer.into_iter().map(|x| x as f32).collect()
}

pub fn jintarray_to_vec(env: &JNIEnv, array: JIntArray) -> Vec<i32> {
    let len = env.get_array_length(&array).unwrap() as usize;
    let mut buffer: Vec<jint> = vec![0; len];
    env.get_int_array_region(array, 0, &mut buffer).unwrap();
    buffer.into_iter().map(|x| x as i32).collect()
}

pub fn from_jint_array_to_vector2_int(env: &JNIEnv, array: JIntArray) -> Vector2<i32> {
    let len = env.get_array_length(&array).unwrap() as usize;
    let mut buffer: Vec<i32> = vec![0; len];
    env.get_int_array_region(array, 0, &mut buffer).unwrap();
    Vector2::new(buffer[0], buffer[1])
}

pub fn from_jfloat_array_to_vector2_float(env: &JNIEnv, array: JFloatArray) -> Vector2<f32> {
    let len = env.get_array_length(&array).unwrap() as usize;
    let mut buffer: Vec<f32> = vec![0.0; len];
    env.get_float_array_region(array, 0, &mut buffer).unwrap();
    Vector2::new(buffer[0], buffer[1])
}
