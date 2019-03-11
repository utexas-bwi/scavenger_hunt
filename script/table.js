
/* 
 * toggle editabiity of cell: makes editable if not editable and vice versa
 *
 * parameters:
 *      span: the span containing the pencil glyphicon that was clicked
 *          expects the nesting to be
 *          <tr>
 *              <td>
 *                  <span> pencil icon </span>
 *              </td>
 *          </tr>
 *      start: the cell to start at when making editable
 *          1 = first child, 3 = second, 5 = third, etc.
 *          start at 1 if you want whole row to be editable
 *
 * preconditions:
 *      - span follows proper nesting (noted above)
 *      - start is a number > 1 and < number of child nodes
 *      - tr is formatted to have two uneditable child nodes at end
 *          (in our case, the pencil and x icons)
 * */

function editable(span, start) {
    const tr = span.parentNode.parentNode;
    if (span.classList.contains("glyphicon-pencil")) {
        for (let i = start; i < tr.childNodes.length - 4; i = i + 2) {
            tr.childNodes[i].setAttribute("contentEditable","true");
            tr.childNodes[i].classList.add("editable");
        }
        span.classList.remove("glyphicon-pencil");
        span.classList.add("glyphicon-ok");
    } else {
        for (let i = start; i < tr.childNodes.length - 4; i = i + 2) {
            tr.childNodes[i].setAttribute("contentEditable","false");
            tr.childNodes[i].classList.remove("editable");
        }
        span.classList.add("glyphicon-pencil");
        span.classList.remove("glyphicon-ok");
    }
}


/* functions to add to task table */

function addRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.classList.remove("hide");
    row.classList.remove("table-line");
    editable(row.childNodes[11].childNodes[1], 1);
    table.appendChild(row);
}

function removeRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    table.deleteRow(cell.parentNode.parentNode.rowIndex);
}

function saveChanges() {
    const table = document.getElementById("task-table");
    // TODO: export data to SQL TaskType database 
}

/* functions to add to hunt table */

function addTask(task) {
    const table = document.getElementById("hunt-table");
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.childNodes[1].textContent = task;
    row.classList.remove("hide");
    row.classList.remove("table-line");
    editable(row.childNodes[5].childNodes[1], 3);
    table.appendChild(row);
}
