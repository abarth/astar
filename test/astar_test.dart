import 'package:astar/astar_2d.dart';
import 'package:test/test.dart';

void main() {
  test('control', () {
    final pathFinder = PathFinder2D(isPassable: (x, y) => true);
    final path = pathFinder
        .findPath(
          Location(0, 0),
          Location(1, 2),
        )!
        .toList();
    expect(path.length, greaterThan(1));
    expect(path.last.x, 1);
    expect(path.last.y, 2);
  });
}
