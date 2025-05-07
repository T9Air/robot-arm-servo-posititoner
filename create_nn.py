import pandas as pd
import tensorflow as tf
from tensorflow import keras
from keras import layers

# Load training data from CSV
# Assumes CSV has columns: servo1, servo2, ..., servoN, pos_x, pos_y, pos_z
data = pd.read_csv('training_data.csv')
# Split into inputs and outputs
# Now the input is the position columns and the output is the servo columns
input_columns = ['pos_x', 'pos_y', 'pos_z']
target_columns = [col for col in data.columns if col.startswith("servo")]
X = data[input_columns].values
y = data[target_columns].values

# Define a simple neural network model
model = keras.Sequential([
    layers.Input(shape=(X.shape[1],)),
    layers.Dense(64, activation='relu'),
    layers.Dense(64, activation='relu'),
    # Adjust output layer to match number of servos
    layers.Dense(y.shape[1])
])

model.compile(
    optimizer='adam',
    loss='mean_squared_error',
    metrics=[
        'mae',
        tf.keras.metrics.RootMeanSquaredError()
    ]
)

# Train the model
model.fit(X, y, epochs=50, batch_size=32, validation_split=0.2)

# Save the model
model.save('servo_position_model.h5')
print("Model training complete and saved as 'servo_position_model.h5'.")

# Convert to a TensorFlow Lite model optimized for RPi 3
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # Enable default quantization
tflite_model = converter.convert()

# Save the optimized TFLite model
with open('servo_position_model.tflite', 'wb') as f:
    f.write(tflite_model)
print("Optimized TFLite model saved as 'servo_position_model.tflite'.")