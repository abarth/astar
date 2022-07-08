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

import 'dart:collection';
import 'package:collection/collection.dart';

/// A neighbor is a node that can be reached from the current node.
class Neighbor<T> {
  /// The node that is the neighbor.
  final T node;

  /// The cost of moving to the neighbor.
  final double cost;

  /// Creates a new neighbor.
  const Neighbor(this.node, this.cost);
}

class _NodeData<T> {
  final T node;
  bool isOpen = false;
  bool isClosed = false;
  double distanceEstimate = double.infinity;
  double approximateDistanceToGoal = double.infinity;
  _NodeData<T>? from;

  _NodeData(this.node);
}

/// A* pathfinding algorithm.
///
/// Override the [getApproximateDistanceToGoal] and [getNeighbors] methods to
/// represent the graph.
///
/// Call [findPath] to find a path from [start] to [goal].
abstract class PathFinder<T> {
  /// Returns an estimate of the distance from [a] to [b].
  ///
  /// This value is used to priortize search nodes that are closer to the goal.
  double getApproximateDistance(T a, T b);

  /// Returns the neighbors of [node].
  ///
  /// Along with each neighbor, this function returns the cost of traversing the
  /// edge from [node] to that neighbor.
  Iterable<Neighbor<T>> getNeighborsOf(T node);

  /// Find a path from [start] to [goal].
  ///
  /// This function returns a list of nodes that represent the path from [start]
  /// to [goal]. If no path is found, this function returns `null`.
  Iterable<T>? findPath(T start, T goal) {
    final open = HeapPriorityQueue<_NodeData<T>>((a, b) =>
        a.approximateDistanceToGoal.compareTo(b.approximateDistanceToGoal));

    final data = HashMap<T, _NodeData<T>>();

    _NodeData<T> getNodeData(T node) {
      return data.putIfAbsent(node, () => _NodeData(node, 0, 0, null));
    }

    void addToOpen(_NodeData<T> data) {
      assert(!data.isOpen);
      assert(!data.isClosed);
      open.add(data..isOpen = true);
    }

    addToOpen(getNodeData(start)
      ..approximateDistanceToGoal = getApproximateDistance(start, goal));

    while (open.isNotEmpty) {
      final currentData = open.removeFirst();

      // If we have found the goal, we can return the path by traversing the
      // from links backwards from this node to the start node.
      if (currentData.node == goal) {
        final path = <T>[];
        for (var data = currentData; data.from != null; data = data.from!) {
          path.add(data.node);
        }
        return path.reversed;
      }

      // Transition this node to the closed state.
      assert(currentData.isOpen);
      assert(!currentData.isClosed);
      currentData.isOpen = false;
      currentData.isClosed = true;

      for (final neighbor in getNeighborsOf(currentData.node)) {
        final neighborData = getNodeData(neighbor.node);

        // We have already visited this node. We cannot update the shortest
        // because that value is already baked distance measurements that
        // traverse this node.
        if (neighborData.isClosed) {
          continue;
        }

        // As long as we have not closed this node, we can update the shortest
        // distance from start so far.
        final distance = currentData.distanceEstimate + neighbor.cost;
        if (distance < neighborData.distanceEstimate) {
          neighborData.from = currentData;
          neighborData.distanceEstimate = distance;
        }

        // If this node is not open, add it to the open list so that we explore
        // paths that traverse this node.
        if (!neighborData.isOpen) {
          addToOpen(neighborData
            ..approximateDistanceToGoal = neighborData.distanceEstimate +
                getApproximateDistance(neighborData.node, goal));
        }
      }
    }
    return null;
  }
}
