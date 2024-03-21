# Power Surge Logger

## Introduction

The Power Surge Logger is a project designed to monitor the electrical voltage in a home and log any power surges detected. Built on the Teensy 4.1 microcontroller and developed in PlatformIO, this project leverages the microcontroller's ADC capabilities for high-frequency monitoring and employs a unique filter for anomaly detection. This endeavor is not just about creating a practical tool for electrical safety but also serves as a deep dive into embedded system design, real-time data processing, and the challenges of accurately detecting electrical anomalies.

## Project Status

This project is in the developmental phase, with current efforts concentrated on optimizing the ADC sampling rate, refining the anomaly detection algorithm within the Stochastic Filter module, and ensuring efficient data logging and storage.

## Design Overview

### SafeQueue Module

The `SafeQueue` module introduces a thread-safe FIFO queue mechanism, engineered to support efficient data handling in concurrent programming environments. Its design is centered around optimizing performance and ensuring robust resource management under the constraints of real-time processing. The module encapsulates several key features and design strategies:

#### Memory Pooling Technique

At the core of `SafeQueue` is a memory pooling technique, which pre-allocates a pool of `Container` objects to minimize the runtime overhead associated with dynamic memory allocation. This strategy not only accelerates enqueue and dequeue operations but also stabilizes latency, making the system's performance more predictable and consistent.

#### Queue Management with Lock-Free Algorithm

- **QueueManager Class:** `SafeQueue` utilizes a variation of the Michael & Scott lock-free queue algorithm for its internal `QueueManager`. This choice ensures thread-safe manipulation of the queue's contents without the need for locking mechanisms, thereby enhancing throughput and reducing potential bottlenecks in multi-threaded applications.
- **Container Structure:** The queue is composed of `Container` objects, each holding a data value (`int16_t`) and a pointer to the next container in the sequence. The atomic nature of the `next` pointer supports the lock-free property of the queue, allowing for safe concurrent access.


### SurgeFilter Module

The `SurgeFilter` is developed to detect power surges in electrical systems, with a focus on simplicity and efficiency. The methodology encompasses:

#### Filtering of the Signal
The chosen strategy is the addition of signal values half a wavelength apart. This efficiently eliminates the 50Hz carrier wave and its harmonics, simplifying the signal to isolate significant deviations that could indicate surges.

#### Statistical Analysis
- **Exponential Moving Averages (EMAs):** Utilized to dynamically track the signal's mean, EMAs offer a streamlined way to monitor the central tendency without the need for extensive historical data. Moreover, updating EMAs is computationally efficient, leveraging the processor's Floating Point Unit to ensure rapid and resource-effective computations.

- **Forgetful Welford's Method:** This approach is a hybrid that combines the robustness of traditional Welford's method for calculating standard deviation with the adaptability of an EMA. The resulting method keeps a current measure of the signal's variability.

- **Aggregation of Z-Scores:** By aggregating Z-scores from various data points, the filter consolidates evidence prior to anomaly detection, improving the reliability of its determinations.

- **Threshold-Based Flagging:** The application of a threshold to aggregated Z-scores is used for distinguishing normal signal behavior from genuine power surges.

#### Ongoing Development
Currently in a developmental stage, the `SurgeFilter` is under continuous refinement to improve both its accuracy and efficiency in real-time surge detection.

#### Future Implementations
Future plans include the introduction of an actual filtering mechanism and possibly a Bayesian approach for the aggregation of Z-scores. These enhancements aim to further advance the filter's capabilities, making it more adaptable to various operational scenarios.

## Learning Outcomes

Engaging with the Power Surge Logger project offers a hands-on experience in several key areas of embedded systems and software engineering, including:

- Advanced sampling techniques and anomaly detection algorithms, focusing on the unique challenges of real-time electrical monitoring.
- Utilizing the PlatformIO ecosystem for embedded software development, exploring its capabilities for cross-platform development and testing.
- In-depth work with the Teensy 4.1 microcontroller, understanding its hardware capabilities, especially in terms of ADC utilization and data storage.
- The application of OOP principles in an embedded context, designing a modular and maintainable codebase.

## Looking Ahead

Future developments for the Power Surge Logger project will include implementing remote monitoring capabilities, enhancing the user interface for easier interaction and data retrieval, and comprehensive documentation covering the system's architecture, setup, and operation.

## Contributing

While this project is a personal venture into embedded systems and electrical monitoring, feedback and insights are always appreciated. Although direct contributions are not currently sought, any interest or suggestions for improvement are welcome.

## License

This project is licensed under the GNU General Public License v3. Detailed licensing information can be found in the [LICENSE](LICENSE) file.
