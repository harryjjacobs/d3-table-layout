# d3-table-layout (WiP)
Grouped node layout with orthogonal links 

## API Reference

### `d3.tableLayout()`
Creates a new table layout with default settings.

### `tableLayout(nodes, links)`
Computes the tables and node positions and returns a graph object containing tables, nodes and links.
[`d3.hierarchy()`](https://github.com/d3/d3-hierarchy#hierarchy) is used to structure the data - so
the nodes and tables will have the same properties - the only difference is that here multiple root
nodes are allowed. The nodes with a depth of zero are used as tables. The nodes with a depth of one
remain as nodes. Nodes with a depth greater than 1 are not supported (the nodes cannot have children).

The returned graph has the following properties:
* tables - the array of tables (top-level nodes)
* nodes - the array of nodes
* links - the array of links

In addition to all the properties from [`d3.hierarchy()`](https://github.com/d3/d3-hierarchy#hierarchy)
resulting nodes will have the following properties:
* x - The x position of the node or table
* y - The y position of the node or table

The tables will have the following properties in addition to those of the nodes, listed above:
* w - The table width (calculated based on the specified `nodeSize`)
* h - The table height (calculated based on the specified `nodeSize` and the number of nodes)

Currently only a single column layout is supported for tables.

## Examples
```javascript
var nodes = [
  {
    name: "Table 1",
  },
  {
    name: "Table 2",
    children: [
      {
        name: "Node a",
      },
      {
        name: "Node b",
      },
    ]
  }
]
```

```javascript
d3.tableLayout().size([width, height]).nodeSize([nodeWidth, nodeHeight])(nodes);
```
