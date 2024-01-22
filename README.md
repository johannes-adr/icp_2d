# icp_2d: Rust ICP Library by Johannes A.

## Introduction

This library provides a Rust implementation of the Iterative Closest Point (ICP) algorithm, commonly used in computer vision and robotics for aligning 2D shapes. It's built for high performance and flexibility, leveraging Rust's strong typing and efficient memory management.
Example use case: Measuring distance and rotation between two lidar-scans.
## Features

- **Efficient Point Matching**: Utilizes KD-Trees for efficient nearest neighbor search.
- **Customizable Point Types**: Supports custom point structures, extending the `ICPPoint` trait.
- **Convergence Control**: Parameters to control convergence based on translation and rotation thresholds.
- **Singular Value Decomposition**: For accurate alignment using the SVD method.

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
your_icp_crate_name = "0.1.0"
```

## Usage

To use the ICP algorithm in your project:

1. Implement the `ICPPoint` trait for your point type.
2. Create instances of `Icp` with reference and other point sets.
3. Use the `do_icp` or `do_icp_once` method to align point sets.

## Example

Here's a basic example of how to use the library:

```rust
use icp_2d::{Icp, ICPPoint};
use nalgebra as na;

// ICPPoint is for na::Point2<f32> implemented
//

// Usage
let ref_points = vec![na::Point2::new(1.0, 2.0), na::Point2::new(3.0, 4.0)];
let other_points = vec![na::Point2::new(5.0, 6.0), na::Point2::new(7.0, 8.0)];

let mut icp = Icp::new(&ref_points, other_points, 50, 0.005, 0.1);
let (x, y, angle) = icp.do_icp(0.0, 0.0, 0.0);

println!("Alignment: Translation ({}, {}), Rotation {}", x, y, angle);
```

## Tip


If your application involves a custom point type solely within the reference points collection (and not in the other points), 
implementing the ICPPoint interface can be more streamlined. In such cases, it's not necessary to implement the translate and rotate methods,
 as the reference point cloud remains stationary.
  This approach simplifies the implementation process, focusing only on the essential functionalities required for stationary reference points in the ICP algorithm.

## Contributing

Contributions to this library are welcome. Here are some ways you can contribute:

- Reporting bugs
- Suggesting enhancements
- Submitting pull requests

---

For more details, please refer to the documentation.
