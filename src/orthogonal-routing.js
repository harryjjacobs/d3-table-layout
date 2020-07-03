import BinaryHeap from "./binary-heap";
const FLOAT_TOLERANCE = 0.0000001;
/**
 * Orthogonal connector routing between points.
 * Loosely based on:
 * Wybrow, Michael, Kim Marriott, and Peter J. Stuckey.
 * "Orthogonal connector routing."
 * International Symposium on Graph Drawing.
 * Springer, Berlin, Heidelberg, 2009.
 */
export default function OrthogonalRouter() {
  this._obstacleCorners = [];
  this._connectorPoints = [];
  this._h = []; // interesting horizontal segments
  this._v = []; // interesting vertical segments
  this._poi = []; // points of interest used 
  this._ovg = []; // orthogonal visibility graph
  this._dirtyAStarNodes = []; // nodes that have been used for A* and need resetting
}

/**
 * 
 * @param {Point[][]} obstacleCorners array containing each set of four corners for each obstacle
 */
OrthogonalRouter.prototype.setObstacles = function(obstacleCorners) {
  this._obstacleCorners = obstacleCorners;
}

/**
 * Points from and to which orthogonal paths can be created
 * @param {Point[]} connectorPoints
 */
OrthogonalRouter.prototype.setConnectorPoints = function(connectorPoints) {
  this._connectorPoints = connectorPoints;
  this._connectorPoints.forEach(c => c.isConnector = true);
}

/**
 * Uses the connector points and obstacles added in order to generate a routing
 * graph which is used by findRoute(). Must be called prior to route finding.
 * @param {Object} area Rect within which the orthogonal paths will be confined
 * @param {number} area.x
 * @param {number} area.y
 * @param {number} area.w
 * @param {number} area.h
 */
OrthogonalRouter.prototype.generateOrthogonalGraph = function(area) {
  let ovg = [];
  // generate poi and sets of interesting segments h and v
  const result = _findInterestingSegments(this._connectorPoints, this._obstacleCorners, area);
  const h = result.h;
  const v = result.v;
  const poi = result.poi;
  this._h = h;
  this._v = v;
  // intersect pairs of segments from H and V
  ovg = _intersectInterestingSegments(h, v);
  this._poi = poi;
  this._ovg = ovg;
  console.log(this._ovg);
}

/**
 * A-star path-finding for orthogonal connectors. Finds a route between
 * two points in the orthogonal visibility graph. OVG must be computed
 * before calling this.
 * Inspiration taken from https://briangrinstead.com/blog/astar-search-algorithm-in-javascript-updated/
 * and https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
 * @param {Point} start ovg connector position
 * @param {Point} end ovg connector position
 */
OrthogonalRouter.prototype.findRoute = function(start, end) {
  // reset nodes used in previous route
  while (this._dirtyAStarNodes.length) {
    let n = this._dirtyAStarNodes.pop();
    n.visited = false;
    n.closed = false;
    n.parent = null;
    n.h = 0;
    n.g = 0;
    n.f = 0;
  }
  // if start and end not in ovg - find their equivalent positions
  if (!start.neighbours) {
    start = this._connectorPoints.find(p => _arePointsEqual(start, p));
  }
  if (!end.neighbours) {
    end = this._connectorPoints.find(p => _arePointsEqual(end, p));
  }
  if (!(end || {}).neighbours || !(start || {}).neighbours) {
    throw "Invalid start and end positions. No connector points available at these positions. Has the OVG been generated?";
  }
  // set of open nodes is a binary heap, sorted by (minimum) f value
  const openNodes = new BinaryHeap(n => n.f);
  // reset first node and add to open set
  start.g = 0;
  start.h = _manhattanHeuristic(start, end);
  openNodes.push(start);
  // mark node to be cleaned before future routing
  this._dirtyAStarNodes.push(start);
  // iterate through open set of nodes, sorted in order of lowest f score
  while (openNodes.size() > 0) {
    // get node with lowest f score
    const currentNode = openNodes.pop();
    if (currentNode === end) {
      // we have reached the end
      return _reconstructAStarPath(currentNode);
    }
    // close the current node now that we have chosen it
    // (equivalent to adding it to a closed set)
    currentNode.closed = true;
    // loop over neighbours
    for (const [_, neighbour] of Object.entries(currentNode.neighbours)) {
      // don't travel through closed nodes or connectors (other than the end connector)
      if (neighbour.closed || (neighbour.isConnector && neighbour !== end)) continue;
      // keep track of whether node has been visited
      // before so that score can be updated in binary
      // heap
      const previouslyVisited = neighbour.visited;
      // g is distance from start node to neighbour, through current
      // pointDistanceSquared is the edge weight between current and this neighbour.
      const tentativeG = currentNode.g + _edgeWeight(currentNode, neighbour);
      if (!previouslyVisited || tentativeG < neighbour.g) {
        neighbour.g = tentativeG;
        // h is the heuristic that estimates the cost of the
        // cheapest path from neighbour node to the end
        neighbour.h = _manhattanHeuristic(neighbour, end);
        // f is the total cost that we want to minimise
        neighbour.f = neighbour.g + neighbour.h;
        // set parent so that the path can be reconstructed later
        neighbour.parent = currentNode;
        neighbour.visited = true;
        // mark node to be cleaned before future routing
        this._dirtyAStarNodes.push(neighbour);
      }
      if (previouslyVisited) {
        // the binary heap needs updating now that the f score
        // is lower - bubble up the newly changed neighbour
        // element to the top
        openNodes.bubbleUpElement(neighbour);
      } else {
        // add to binary heap (and automatically sort it)
        openNodes.push(neighbour);
      }
    }
  }
  // did not reach the end point
  return false;
}

function _reconstructAStarPath(endNode) {
  let path = [endNode];
  while (endNode.parent) {
    endNode = endNode.parent;
    path.push(endNode);
  }
  return path.reverse();
}

function _manhattanHeuristic(a, b) {
  var distanceX = Math.abs(b.x - a.x);
  var distanceY = Math.abs(b.y - a.y);
  return distanceX + distanceY;
}

/**
 * Weight of travelling along an edge
 * between two nodes, for A* path finding.
 * Based on euclidean distance and also 
 * uses the parent property of each
 * node to add a cost for turns.
 * @param {Object} current 
 * @param {Object} neighbour 
 */
function _edgeWeight(current, neighbour) {
  let weight = (current.x - neighbour.x) * (current.x - neighbour.x) +
    (current.y - neighbour.y) * (current.y - neighbour.y);

  if (current.parent) {
    let currentDir = direction(current.parent, current);
    let newDir = direction(current, neighbour);
    if (currentDir !== newDir) {
      weight += 0.1;
    }
  }

  return weight;
}

/**
 * Determines the direction between two points
 * represented as 0, 1, 2, 3 for north, east,
 * south, west. -1 for unknown.
 * @param {Point} a 
 * @param {Point} b 
 */
export function direction(a, b) {
  if ((b.x - a.x) > FLOAT_TOLERANCE) {
    return 1; // east
  } else if ((b.x - a.x) < -FLOAT_TOLERANCE) {
    return 3; // west
  } else if ((b.y - a.y) > FLOAT_TOLERANCE) {
    return 2; // south
  } else if ((b.y - a.y) < -FLOAT_TOLERANCE) {
    return 0; // north
  }

  return -1;
}

/**
 * Assembles point of interest (poi) from the connector points and obstacles' corners.
 * Travels north, south, east, west from a point until a table is intersected or the edge of the layout is hit
 * Returns the resulting h and v segments
 * @param {Point[]} connectorPoints The connector points (and any additional points of interest - excluding obstacle corners.)
 * @param {Point[][]} obstaclesCorners The obstacle's corners
 * @param {Object} area Rect in which to find segments. Area of the layout.
 * @param {number} area.x
 * @param {number} area.y
 * @param {number} area.w
 * @param {number} area.h
 * @returns {Object} object containing h and v segments
 */
function _findInterestingSegments(connectorPoints, obstaclesCorners, area) {
  let poi = [...connectorPoints];
  let h = [];
  let v = [];
  // find segments between table corners first and then add to poi
  obstaclesCorners.forEach(corners => {
    const tl = corners[0];
    const tr = corners[1];
    const bl = corners[2];
    const br = corners[3];
    // neighbours clockwise from north
    tl.neighbours = { east: tr, south: bl };
    tr.neighbours = { west: tl, south: br };
    bl.neighbours = { north: tl, east: br };
    br.neighbours = { north: tr, west: bl };
    // Add the obstacle corners as points of interest
    poi.push(tl, tr, bl, br);
    // Create horizontal and vertical segments for table edges
    // as they will be skipped during the process below due
    // to the manual neighbour assignment above.
    v.push({ a: tl, b: bl }, { a: tr, b: br });
    h.push({ a: tl, b: tr }, { a: bl, b: br });
  });
  // find horizontal and vertical segments for each point of interest
  poi.forEach(point => {
    // define ordinal extremities
    let north = area.y;
    let south = area.y + area.h;
    let west = area.x;
    let east = area.x + area.w;
    if (!point.neighbours) point.neighbours = {};
    obstaclesCorners.forEach(obstCorners => {
      // North
      if (!point.neighbours.north && !point.horizontalOnly) {
        // point does not yet have a northerly neighbour, so find one
        if (point.x >= obstCorners[0].x && point.x <= obstCorners[1].x && point.y > obstCorners[2].y) {
          // point in x-range of table and bottom table edge is above point
          if (obstCorners[2].y > north) {
            // smaller distance to table edge found
            north = obstCorners[2].y;
          }
        }
      }
      // South
      if (!point.neighbours.south && !point.horizontalOnly) {
        // point does not yet have a southerly neighbour, so find one
        if (point.x >= obstCorners[0].x && point.x <= obstCorners[1].x && point.y < obstCorners[0].y) {
          // point in x-range of table and top table edge is below point
          if (obstCorners[0].y < south) {
            // smaller distance to table edge found
            south = obstCorners[0].y;
          }
        }
      }
      // East
      if (!point.neighbours.east && !point.verticalOnly) {
        // point does not yet have a easterly neighbour, so find one
        if (point.y >= obstCorners[0].y && point.y <= obstCorners[2].y && point.x < obstCorners[0].x) {
          // point in y-range of table and top table edge is below point
          if (obstCorners[0].x < east) {
            // smaller distance to table edge found
            east = obstCorners[0].x;
          }
        }
      }
      // West
      if (!point.neighbours.west && !point.verticalOnly) {
        // point does not yet have a easterly neighbour, so find one
        if (point.y >= obstCorners[0].y && point.y <= obstCorners[2].y && point.x > obstCorners[1].x) {
          // point in y-range of table and top table edge is below point
          if (obstCorners[1].x > west) {
            // smaller distance to table edge found
            west = obstCorners[1].x;
          }
        }
      }
    });
    if (!point.neighbours.north && !point.horizontalOnly) {
      let northNeighbour = { x: point.x, y: north, neighbours: { south: point } };
      point.neighbours.north = northNeighbour;
      v.push({ a: point, b: northNeighbour });
    }
    if (!point.neighbours.south && !point.horizontalOnly) {
      let southNeighbour = { x: point.x, y: south, neighbours: { north: point } };
      point.neighbours.south = southNeighbour;
      v.push({ a: point, b: southNeighbour });
    }
    if (!point.neighbours.east && !point.verticalOnly) {
      let eastNeighbour = { x: east, y: point.y, neighbours: { west: point } };
      point.neighbours.east = eastNeighbour;
      h.push({ a: point, b: eastNeighbour });
    }
    if (!point.neighbours.west && !point.verticalOnly) {
      let westNeighbour = { x: west, y: point.y, neighbours: { east: point } };
      point.neighbours.west = westNeighbour;
      h.push({ a: point, b: westNeighbour });
    }
  });
  return { h: h, v: v, poi: poi };
}

function _intersectInterestingSegments(h, v) {
  let ovg = [];
  for (let hIndex = 0; hIndex < h.length; hIndex++) {
    for (let vIndex = 0; vIndex < v.length; vIndex++) {
      // hIndex can change during progress of this inner loop
      // so grab a reference here each time instead of in outer loop
      const hSeg = h[hIndex];
      const vSeg = v[vIndex];
      let intersection = _intersectLineSegments(hSeg, vSeg);
      if (intersection) {
        // check if intersection is at end of h line segment - if so
        // use that point instead of creating a new one
        if (_arePointsEqual(intersection, hSeg.a)) intersection = hSeg.a;
        if (_arePointsEqual(intersection, hSeg.b)) intersection = hSeg.b;
        // ignore intersections at segment corners.
        // we can safely assume that the shared points
        // in these two segments are already neighbours.
        if (_areSegmentsConnected(hSeg, vSeg)) continue;
        // begin assigning new neighbours generated by the
        // intersection
        intersection.neighbours = {};
        if (hSeg.a.x < intersection.x) {
          // split vertical line segment into two segments
          const leftSeg = { a: hSeg.a, b: intersection };
          const rightSeg = { a: intersection, b: hSeg.b };
          // remove original segment from vertical list
          // and replace with the two new ones
          h.splice(hIndex, 1, leftSeg, rightSeg);
          hIndex--; // skip behind the newly inserted items
          hSeg.a.neighbours.east = intersection;
          intersection.neighbours.west = hSeg.a;
          hSeg.b.neighbours.west = intersection;
          intersection.neighbours.east = hSeg.b;
        } else {
          // split vertical line segment into two segments
          const leftSeg = { a: intersection, b: hSeg.b };
          const rightSeg = { a: hSeg.a, b: intersection };
          // remove original segment from vertical list
          // and replace with the two new ones
          h.splice(hIndex, 1, leftSeg, rightSeg);
          hIndex--; // skip behind the newly inserted items
          hSeg.a.neighbours.west = intersection;
          intersection.neighbours.east = hSeg.a;
          hSeg.b.neighbours.east = intersection;
          intersection.neighbours.west = hSeg.b;
        }
        if (vSeg.a.y > intersection.y) {
          // split vertical line segment into two segments
          const topSeg = { a: intersection, b: vSeg.b };
          const bottomSeg = { a: vSeg.a, b: intersection };
          // remove original segment from vertical list
          // and replace with the two new ones
          v.splice(vIndex, 1, topSeg, bottomSeg);
          vIndex++; // skip past the newly inserted items
          vSeg.a.neighbours.north = intersection;
          intersection.neighbours.south = vSeg.a;
          vSeg.b.neighbours.south = intersection;
          intersection.neighbours.north = vSeg.b;
        } else {
          // split vertical line segment into two segments
          const topSeg = { a: vSeg.a, b: intersection };
          const bottomSeg = { a: intersection, b: vSeg.b };
          // remove original segment from vertical list
          // and replace with the two new ones
          v.splice(vIndex, 1, topSeg, bottomSeg);
          vIndex++; // skip past the newly inserted items
          vSeg.a.neighbours.south = intersection;
          intersection.neighbours.north = vSeg.a;
          vSeg.b.neighbours.north = intersection;
          intersection.neighbours.south = vSeg.b;
        }
        ovg.push(intersection);
      }
    }
  }
  return ovg;
}

/**
 * From Paul Bourke http://paulbourke.net/geometry/pointlineplane/
 * Determine the intersection point of two line segments.
 * Return FALSE if the lines don't intersect.
 * @param {Object} seg1 Line segment 1
 * @param {Point} seg1.a Starting point of segment 1
 * @param {Point} seg1.b Finishing point of segment 1
 * @param {Object} seg2 Line segment 1
 * @param {Point} seg2.a Starting point of segment 2
 * @param {Point} seg2.b Finishing point of segment 2
 */
function _intersectLineSegments(seg1, seg2) {
  // Check none of the lines are of length 0
  if ((seg1.a.x === seg1.b.x && seg1.a.y === seg1.b.y) || (seg2.a.x === seg2.b.x && seg2.a.y === seg2.b.y)) {
    return false;
  }

  let denominator = ((seg2.b.y - seg2.a.y) * (seg1.b.x - seg1.a.x) - (seg2.b.x - seg2.a.x) * (seg1.b.y - seg1.a.y));

  // Lines are parallel
  if (denominator === 0) {
    return false;
  }

  let ua = ((seg2.b.x - seg2.a.x) * (seg1.a.y - seg2.a.y) - (seg2.b.y - seg2.a.y) * (seg1.a.x - seg2.a.x)) / denominator;
  let ub = ((seg1.b.x - seg1.a.x) * (seg1.a.y - seg2.a.y) - (seg1.b.y - seg1.a.y) * (seg1.a.x - seg2.a.x)) / denominator;

  // Is the intersection along the segments?
  if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
    return false;
  }

  // Return the x and y coordinates of the intersection
  let x = seg1.a.x + ua * (seg1.b.x - seg1.a.x);
  let y = seg1.a.y + ua * (seg1.b.y - seg1.a.y);

  return { x, y };
}

/**
 * Determines whether two segments have a shared endpoint
 * @param {Object} seg1 segment 1
 * @param {Object} seg2 segment 2
 */
function _areSegmentsConnected(seg1, seg2) {
  return _arePointsEqual(seg1.a, seg2.a) ||
    _arePointsEqual(seg1.a, seg2.b) ||
    _arePointsEqual(seg1.b, seg2.a) ||
    _arePointsEqual(seg1.b, seg2.b);
}

/**
 * 
 * @param {Point} p1 
 * @param {Point} p2 
 */
function _arePointsEqual(p1, p2) {
  return (Math.abs(p1.x - p2.x) < FLOAT_TOLERANCE) && (Math.abs(p1.y - p2.y) < FLOAT_TOLERANCE);
}

/**
 * The point type
 *
 * @typedef {Object} Point
 * @param {number} x
 * @param {number} y
 */