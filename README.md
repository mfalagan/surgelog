# Power Surge Logger

## Introduction

The Power Surge Logger is a project designed to monitor the electrical voltage in a home and log any power surges detected. Built on the Teensy 4.1 microcontroller and developed in PlatformIO, this project leverages the microcontroller's ADC capabilities for high-frequency monitoring and employs a unique filter for anomaly detection. This endeavor is not just about creating a practical tool for electrical safety but also serves as a deep dive into embedded system design, real-time data processing, and the challenges of accurately detecting electrical anomalies.

## Project Status

This project is in the developmental phase, with current efforts concentrated on optimizing the ADC sampling rate, refining the anomaly detection algorithm within the Stochastic Filter module, and ensuring efficient data logging and storage.

## Technical Highlights

### SafeQueue Memory Pooling

The `SafeQueue` thread-safe FIFO queue employs a memory pooling technique to enhance performance and ensure efficient resource management. By pre-allocating a pool of `Container` objects, the queue minimizes dynamic memory allocation overhead during runtime, which is crucial for maintaining high performance in real-time data processing. This approach not only speeds up enqueue and dequeue operations but also provides more predictable latency by avoiding frequent calls to the allocator.

### SurgeFilter Efficiency

The `SurgeFilter` is engineered for high-efficiency detection of power surges by simplifying traditional signal processing methods. It isolates anomalies through subtraction of signal values half a wavelength apart, effectively neutralizing the carrier wave and focusing on significant deviations. Initially calibrated for a 50Hz baseline with plans for adaptive optimization, this method balances precision and processing speed. The filter operates in real-time, dynamically adjusting to detect surges against a backdrop of normal fluctuations, ensuring minimal latency and resource use. Integrated within a continuous monitoring loop, it pauses only to document detected surges.

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
