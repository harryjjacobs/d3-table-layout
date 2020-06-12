import { default as rectCollide } from "./collide-rect";

export default function() {
  var width = 1,
      height = 1,
      nodeWidth = 1,
      nodeHeight = 1,
      tableSeparation = 10;
  // Use top-most nodes in force directed graph
  // Then populate them with their children
  // Then route links between child nodes

  function tableLayout(nodes, links) {
    const layout = {tables: nodes, nodes: {}, links: links};
    computeFlattenedHierarchy(layout);
    computeForceDirectedTableLayout(layout);
    computeNodePositionsInTables(layout);
    // TODO: compute links
    console.log(layout);
    return layout;
  }

  tableLayout.size = function(_) {
    return arguments.length ? (width = _[0], height = _[1], tableLayout) : [width, height]
  }

  tableLayout.nodeSize = function(_) {
    return arguments.length ? (nodeWidth = _[0], nodeHeight = _[1], tableLayout) : [nodeWidth, nodeHeight]
  }

  tableLayout.tableSeparation = function(_) {
    return arguments.length ? (tableSeparation = _, tableLayout) : tableSeparation
  }

  function computeFlattenedHierarchy(layout) {
    // A root node must be created as an ancestor 
    // to the provided nodes in order to use hierarchy()
    var root = {
      id: "root",
      children: layout.tables
    };
    // Organise into a hierarchical structure
    root = d3.hierarchy(root)
    // Remove root node and shift its children up a
    // level so that tables are top-level and nodes
    // are below tables
    var tables = root.children;
    tables.forEach(table => {
      table.parent = null;
      table.descendants().forEach(n => n.depth -= 1);
    });
    layout.tables = tables;
    layout.nodes = layout.tables.flatMap(t => t.children);
  }

  function computeForceDirectedTableLayout(layout) {
    // Apply force simulation to each of the tables in order
    // to distribute them evenly. Perform collision handling
    // using the bounding box of the table rather than a 
    // radius, for better positioning
    var sim = d3.forceSimulation(layout.tables)
    // TODO: use forceLink to move tables with linked nodes closer together
      .force('link', d3.forceLink().id(d => d.id))
      //.force("collide", d3.forceCollide().radius(function(d) { return Math.max(computeTableWidth(d), computeTableHeight(d)) / 2.0 + tableSeparation; }))
      .force("collide", rectCollide().size(d => [computeTableWidth(d) + tableSeparation, computeTableHeight(d) + tableSeparation]))
      .force("charge", d3.forceManyBody())
      .force("center", d3.forceCenter(width / 2.0, height / 2.0))
      .stop();
    // Run simulation for x ticks in order to position the tables
    sim.tick(300);
    // Adjust table positions as position from force simulation
    // is at the center of the object
    layout.tables.forEach(t => {
      t.x -= computeTableWidth(t) / 2.0;
      t.y -= computeTableHeight(t) / 2.0;
    });
  }

  function computeNodePositionsInTables(layout) {
    // Position each table's nodes
    layout.tables.forEach(table => {
      var n = ('children' in table) ? table.children.length : 0;
      var y = table.y;
      var x = table.x;
      for (let i = 0; i < n; i++) {
        const child = table.children[i];
        child.x = x;
        child.y = y;
        y += nodeHeight;
      }
      table.w = nodeWidth;
      table.h = nodeHeight * n;
    });
  }

  function computeTableHeight(table) {
    return ('children' in table) ? (nodeHeight * table.children.length) : 0;
  }

  function computeTableWidth() {
    return nodeWidth;
  }

  return tableLayout;
};
