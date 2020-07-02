import "d3-hierarchy";
import "d3-force";
import { default as bboxCollide } from "./bbox-collide";
import OrthogonalRouter from "./orthogonal-routing";

export default function() {
  var width = 50,
    height = 50,
    nodeWidth = 10,
    nodeHeight = 5,
    tableInflation = 5, // margin around table for layout and routing
    pathShift = 3; // amount to translate path points by in x and y to avoid overlap;

  function tableLayout(nodes, links) {
    const layout = { tables: {}, nodes: nodes, links: links || [] };
    _computeTableNodeHierarchy(layout);
    _computeForceDirectedTableLayout(layout);
    _computeNodePositionsInTables(layout);
    _computeNodeLinks(layout);
    _computeLinkPoints(layout);
    return layout;
  }

  /**
   * Sets the size within which to layout the tables
   * @param {Number[]} size [width, height]
   */
  tableLayout.size = function(size) {
    return arguments.length ? (width = size[0], height = size[1], tableLayout) : [width, height]
  }

  /**
   * Sets the size of the nodes contained in each table
   * @param {Number[]} nodeSize [x, y]
   */
  tableLayout.nodeSize = function(nodeSize) {
    return arguments.length ? (nodeWidth = nodeSize[0], nodeHeight = nodeSize[1], tableLayout) : [nodeWidth, nodeHeight]
  }

  /**
   * Sets the inflation margin around each table - used for layout and routing purposes.
   * Table inflation determines table separation when laying out and also connector edge
   * positioning when creating links.
   * @param {Number} inflation 
   */
  tableLayout.tableInflation = function(inflation) {
    return arguments.length ? (tableInflation = inflation, tableLayout) : tableInflation
  }

  function _computeTableNodeHierarchy(layout) {
    // A root node must be created as an ancestor 
    // to the provided nodes in order to use hierarchy()
    var root = {
      id: "root",
      children: layout.nodes
    };
    // Organise into a hierarchical structure
    root = d3.hierarchy(root)
      // Remove root node and shift its children up a
      // level so that tables are top-level and nodes
      // are below tables
    var tables = root.children;
    tables.forEach(table => {
      table.parent = null; // tables are top-level
      delete table.height; // unnecessary and would cause confusion with table rect height
      table.descendants().forEach(n => n.depth -= 1);
    });
    layout.tables = tables;
    layout.nodes = layout.tables.flatMap(t => t.children);
  }

  function _computeForceDirectedTableLayout(layout) {
    // Apply force simulation to each of the tables in order
    // to distribute them evenly. Perform collision handling
    // using the bounding box of the table rather than a 
    // radius, for better positioning
    const collide = bboxCollide(d => {
      console.log(d);
      const width = _computeInflatedTableWidth(d);
      const height = _computeInflatedTableHeight(d);
      return [
        [-width / 2 - 5, -height / 2 - 5],
        [width / 2 + 5, height / 2 + 5]
      ];
    });
    var sim = d3.forceSimulation(layout.tables)
      // TODO: use forceLink to move tables with linked nodes closer together
      //.force('link', d3.forceLink().id(d => d.id))
      //.force("collide", d3.forceCollide().radius(function(d) { return Math.max(computeTableWidth(d), computeTableHeight(d)) / 2.0 + tablePadding; }))
      //.force("collide", rectCollide().size(d => [_computeInflatedTableWidth(d), _computeInflatedTableWidth(d)]))
      .force("collide", collide)
      .force("charge", d3.forceManyBody().strength(1000))
      .force("center", d3.forceCenter(width / 2.0, height / 2.0))
      .stop();
    // Run simulation for x ticks in order to position the tables
    for (var i = 0; i < 300; ++i) {
      // constrain each table within the layout area
      layout.tables.forEach(table => {
        const tableWidthExtent = _computeInflatedTableWidth(table) / 2 + 10;
        const tableHeightExtent = _computeInflatedTableHeight(table) / 2 + 10;
        table.x = Math.min(Math.max(table.x, tableWidthExtent), width - tableWidthExtent);
        table.y = Math.min(Math.max(table.y, tableHeightExtent), height - tableHeightExtent);
      });
      sim.tick();
    }
    //sim.tick(300);
    // Adjust table positions as position from force simulation
    // is at the center of the object
    layout.tables.forEach(t => {
      var width = _computeTableWidth(t);
      var height = _computeTableHeight(t);
      t.x -= width / 2.0;
      t.y -= height / 2.0;
      t.w = width;
      t.h = height;
      t.inflatedRect = _computeInflatedTableRect(t);
    });
  }

  function _computeNodePositionsInTables(layout) {
    // Position each table's nodes
    layout.tables.forEach(table => {
      let n = ('children' in table) ? table.children.length : 0;
      let y = table.y;
      let x = table.x;
      for (let i = 0; i < n; i++) {
        const child = table.children[i];
        child.x = x;
        child.y = y;
        y += nodeHeight;
      }
    });
  }

  function _computeNodeLinks(layout) {
    // each node will store incoming (target) and outgoing (source) links
    layout.nodes.forEach((node, i) => {
      node.index = i;
      node.sourceLinks = [];
      node.targetLinks = [];
    });
    // convert link source and target from indices to node references
    layout.links.forEach((link, i) => {
      link.index = i;
      console.log(link);
      if (typeof link.source !== "object") link.source = layout.nodes.find(n => n.index === link.source);
      if (typeof link.target !== "object") link.target = layout.nodes.find(n => n.index === link.target);
      link.source.sourceLinks.push(link);
      link.target.targetLinks.push(link);
    });
  }

  function _computeLinkPoints(layout) {
    const router = new OrthogonalRouter(width, height);
    const obstacleCorners = layout.tables.map(t => t.inflatedRect.corners);
    const connectorPoints = layout.nodes.map(n => _computeConnectorPoint(n));

    router.setObstacles(obstacleCorners);
    router.setConnectorPoints(connectorPoints);

    let t0 = performance.now();
    // compute orthogonal visibility graph
    router.generateOrthogonalGraph({ x: 0, y: 0, w: width, h: height });
    let t1 = performance.now();
    console.log(`Generating OVG took ${(t1 - t0).toFixed(2)}ms`);

    // route a path for all of the (visible) links
    // and shift points in link by a small amount
    // to avoid overlap
    let pathShiftAmount = 0;
    layout.links.forEach(link => {
      t0 = performance.now();
      // find route using A* path finder
      // copy points to new array of 
      // clean points
      link.points = router.findRoute(
        _computeConnectorPoint(link.source),
        _computeConnectorPoint(link.target)
      ).map(p => ({ x: p.x, y: p.y }));
      t1 = performance.now();
      console.log(`A* route finder took ${(t1 - t0).toFixed(2)}ms for link`, link);
      _performPathShift(link.points, { x: pathShiftAmount, y: pathShiftAmount });
      pathShiftAmount += pathShift;
    });

    console.log(layout.links);

    layout.poi = router._poi;
    layout.ovg = router._ovg;
    layout.h = router._h;
    layout.v = router._v;
  }

  /**
   * Translates points in an orthogonal path such that the first and last
   * points remain the same and the others are shifted by the translation 
   * where possible (in order for the path to remain orthogonal).
   * @param {Point[]} path 
   * @param {Point} translation 
   */
  function _performPathShift(path, translation) {
    for (let i = 0; i < path.length; i++) {
      const point = path[i];
      if (i == 1) {
        if (point.x === path[0].x) {
          point.y += translation.y;
        } else if (point.y === path[0].y) {
          point.x += translation.x;
        }
      }
      if (i == path.length - 2) {
        if (point.x === path[path.length - 1].x) {
          point.y += translation.y;
        } else if (point.y === path[path.length - 1].y) {
          point.x += translation.x;
        }
      }
      if (i > 1 && i < path.length - 2) {
        point.x += translation.x;
        point.y += translation.y;
      }
    }
  }

  function _computeConnectorPoint(node) {
    return { x: node.x + nodeWidth / 2, y: node.y + nodeHeight / 2, horizontalOnly: true };
    //return { x: node.x, y: node.y + nodeHeight / 2, horizontalOnly: true };
  }

  function _computeTableHeight(table) {
    return (('children' in table) ? (nodeHeight * table.children.length) : 0);
  }

  function _computeTableWidth() {
    return nodeWidth;
  }

  function _computeInflatedTableHeight(table) {
    return (('children' in table) ? (nodeHeight * table.children.length + tableInflation * 2) : 0);
  }

  function _computeInflatedTableWidth() {
    return nodeWidth + tableInflation * 2;
  }

  /**
   * Inflates the table size and provides corner points.
   * Used for layout and orthogonal connector
   * @param {*} table 
   */
  function _computeInflatedTableRect(table) {
    const x = table.x - tableInflation;
    const y = table.y - tableInflation;
    const w = table.w + tableInflation * 2;
    const h = table.h + tableInflation * 2;
    const corners = [
      { x: x, y: y }, // tl
      { x: x + w, y: y }, // tr
      { x: x, y: y + h }, // bl
      { x: x + w, y: y + h }, // br
    ];
    return {
      x: x,
      y: y,
      w: w,
      h: h,
      corners: corners
    }
  }
  return tableLayout;
};