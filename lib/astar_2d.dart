// Copyright 2022 Adam Barth
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

library astar;

import 'dart:math' as math;

import 'astar.dart';

/// A location in a 2D grid.
class Location {
  /// The x coordinate.
  final int x;

  /// The y coordinate.
  final int y;

  /// Creates a new location.
  const Location(this.x, this.y);

  /// The distance from this location to [other].
  ///
  /// This is the Euclidean distance.
  double distanceTo(Location other) {
    final dx = x - other.x;
    final dy = y - other.y;
    return math.sqrt(dx * dx + dy * dy);
  }

  @override
  bool operator ==(Object other) =>
      identical(this, other) ||
      other is Location &&
          runtimeType == other.runtimeType &&
          x == other.x &&
          y == other.y;

  @override
  int get hashCode => Object.hash(x, y);

  @override
  String toString() => 'Location($x, $y)';
}

/// Whether the given location is passable in the grid.
typedef IsPassableCallback = bool Function(int x, int y);

/// A* pathfinding algorithm in a 2D grid.
///
/// See [PathFinder] for pathfinding in an arbitrary graph.
class PathFinder2D extends PathFinder<Location> {
  /// Whether the given location is passable in the grid.
  ///
  /// This is used to determine which locations are valid to move to. This
  /// function is responsible for bounds checking.
  final IsPassableCallback isPassable;

  /// Whether to allow diagonal movement.
  final bool allowDiagonal;

  /// Creates a new pathfinder.
  PathFinder2D({required this.isPassable, this.allowDiagonal = true});

  @override
  double getApproximateDistance(Location a, Location b) => a.distanceTo(b);

  @override
  Iterable<Neighbor<Location>> getNeighborsOf(Location node) sync* {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        if (!allowDiagonal && dx != 0 && dy != 0) {
          continue;
        }
        final x = node.x + dx;
        final y = node.y + dy;
        if (isPassable(x, y)) {
          yield Neighbor(Location(x, y), 1.0);
        }
      }
    }
  }
}
