/****************************************************************
*         Hamilton Circuit, use branch and bound method         *
****************************************************************/
// usage:
//     // (a, b) means the distance from node a to node b
//     let matrix = [[ (1, 1), (1, 2), ..., (1, n) ],
//                   [ (2, 1), (2, 2), ..., (2, n) ],
//                   [ ... ],
//                   [ (n, 1), (n, 2), ..., (n, n) ]];
//
//     let optimalPath = Calculator.getHamiltonCircuit(matrix);

class Calculator {

    static getHamiltonCircuit(m) {
        let hamiltonCircuitCalculator = new HamiltonCircuitCalculator();

        return hamiltonCircuitCalculator.getHamiltonCircuit(m);
    };

};

class HamiltonCircuitCalculator {

    #MAX_VAL = 4294967295;
    #DUMMY = -1;

    #pLowerBound = this.#MAX_VAL;
    #pOptimalPath = [];
    #pCurrentOptimalPath = [];

    getHamiltonCircuit(m) {
        let matrix = this.#getMatrixCopy(m);
        let axes = this.#getInitNodeStack(matrix);
        this.#initOptimalPath(matrix.length);
        this.#setNoStandingStill(matrix);
        this.#startChoose(axes.rows, axes.cols, matrix);
        return this.#getOptimalPath();
    }

    #getOptimalPath(startFrom = 0) {
        let optPath = []
        for (let i = 0; i != this.#pOptimalPath.length; i += 1)
            optPath.push(this.#pOptimalPath[i]);

        let path = [];
        let curr = startFrom;
        let next;

        while (curr != this.#DUMMY) {
            path.push(curr);
            next = optPath[curr];
            optPath[curr] = this.#DUMMY;
            curr = next;
        }

        return path;
    }

    #startChoose(rows, cols, matrix, lowerBound = 0) {
        if (lowerBound >= this.#pLowerBound)
            return;

        lowerBound += this.#lowerBoundProcess(matrix);

        if (lowerBound >= this.#pLowerBound)
            return;

        let arcInfo = this.#findBestAdvantageArc(matrix);
        
        let chooseArc = this.#chooseArc(arcInfo, rows, cols, matrix);
        
        if (rows.length == 1) {    // achieve the goal
            // console.log(`final choose: ${rows[arcInfo.rowIndex]}, ${cols[arcInfo.colIndex]}`);
            // console.log(`remove: ${rows[arcInfo.rowIndex]}, ${cols[arcInfo.colIndex]}`);
            lowerBound += arcInfo.cost;
            
            if (lowerBound >= this.#pLowerBound)
                return;
            
            this.#pLowerBound = lowerBound;

            for (let i = 0; i != this.#pOptimalPath.length; i += 1)
                this.#pOptimalPath[i] = this.#pCurrentOptimalPath[i];
            
            this.#pOptimalPath[rows[arcInfo.rowIndex]] = cols[arcInfo.colIndex];

            return;
            
        }

        let isChooseArcPossible =
            this.#isChooseArcPossible(rows[arcInfo.rowIndex], cols[arcInfo.colIndex]);
        
        let isDoNotChooseArcPossible =
            this.#isDoNotChooseArcPossible(rows[arcInfo.rowIndex], cols[arcInfo.colIndex]);

        if (!(isChooseArcPossible || isDoNotChooseArcPossible))
            return;

        if (isChooseArcPossible) {    // choose arc
            // console.log(`choose: ${rows[arcInfo.rowIndex]}, ${cols[arcInfo.colIndex]}`);
            lowerBound += arcInfo.cost;
            this.#pCurrentOptimalPath[rows[arcInfo.rowIndex]] = cols[arcInfo.colIndex];
            this.#startChoose(chooseArc.rows, chooseArc.cols, chooseArc.matrix, lowerBound);
            this.#pCurrentOptimalPath[rows[arcInfo.rowIndex]] = this.#DUMMY;
            lowerBound -= arcInfo.cost;
            // console.log(`remove: ${rows[arcInfo.rowIndex]}, ${cols[arcInfo.colIndex]}`);
        }
        
        if (isDoNotChooseArcPossible) {  // choose no arc
            if (arcInfo.opportunityCost + lowerBound >= this.#pLowerBound)
                return;
            
            matrix[arcInfo.rowIndex][arcInfo.colIndex] = this.#MAX_VAL;
            
            this.#startChoose(rows, cols, matrix, lowerBound);
        }

    }

    #isDoNotChooseArcPossible(from, to) {
        let size = this.#pCurrentOptimalPath.length;

        let flag = false;
        let startFrom, endOf;
        for (endOf = 0; endOf != size; endOf += 1) {
            if (endOf == to)
                continue;
            
            if (this.#isChooseArcPossible(from, endOf)) {
                flag = true;
                break;
            }
        }

        if (!flag)
            return false;

        for (startFrom = 0; startFrom != size; startFrom += 1) {
            if (startFrom == from)
                continue;
            
            if (this.#isChooseArcPossible(startFrom, to))
                return true;
        }

        return false;
    }

    #isChooseArcPossible(from, to) {
        let size = this.#pCurrentOptimalPath.length;
        
        if (this.#pCurrentOptimalPath[from] != this.#DUMMY)
            return false;
            
        for (let val of this.#pCurrentOptimalPath) {
            if (val == to)
                return false;
        }

        let group = [];
        for (let i = 0; i != size; i += 1)
            group.push(i);
        
        
        let findGroup = idx => group[idx] == idx ? idx : findGroup(group[idx]);

        let min, max, minGroup, maxGroup, tmp;
        for (let i = 0; i != size; i += 1) {
            if (this.#pCurrentOptimalPath[i] == this.#DUMMY)
                continue;

            min = Math.min(i, this.#pCurrentOptimalPath[i]);
            max = Math.max(i, this.#pCurrentOptimalPath[i]);
            minGroup = findGroup(min);
            maxGroup = findGroup(max);

            if (minGroup == maxGroup)
                console.log("loop exist: " + min + ", " + max);
            
            if (minGroup > maxGroup) {
                tmp = minGroup;
                minGroup = maxGroup;
                maxGroup = tmp;
            }

            group[maxGroup] = minGroup;
        }

        return findGroup(from) != findGroup(to);
    }

    #chooseArc(arcInfo, rows, cols, matrix) {
        let newRows = [];
        let newCols = [];
        let newMatrix = [];

        let rowLen = rows.length;
        let colLen = cols.length;
        let row, col;
        for (row = 0; row != rowLen; row += 1) {
            if (row == arcInfo.rowIndex)
                continue;
            newRows.push(rows[row]);
        }

        for (col = 0; col != colLen; col += 1) {
            if (col == arcInfo.colIndex)
                continue;
            newCols.push(cols[col]);
        }

        for (row = 0; row != rowLen; row += 1) {
            if (row == arcInfo.rowIndex)
                continue;

            let tmpRow = [];

            for (col = 0; col != colLen; col += 1) {
                if (col == arcInfo.colIndex)
                    continue;
                tmpRow.push(matrix[row][col]);
            }

            newMatrix.push(tmpRow);
        }

        return {
            matrix: newMatrix,
            rows: newRows,
            cols: newCols
        };
    }

    #findBestAdvantageArc(matrix) {
        let height = matrix.length;
        let width = matrix[0].length;
        let min = this.#getFirstTwoSmallestValues(matrix);
        let bestArcLowerBound = { x: -1, y: -1, cost: this.#MAX_VAL, opportunityCost: 0 };

        let row, col, cost, opportunityCost;
        for (row = 0; row != height; row += 1) {
            for (col = 0; col != width; col += 1) {
                cost = matrix[row][col];

                if (cost > bestArcLowerBound.cost)
                    continue;

                opportunityCost =
                    cost == min.row[row].first ? min.row[row].second : min.row[row].first;
                opportunityCost +=
                    cost == min.col[col].first ? min.col[col].second : min.col[col].first;

                if (cost < bestArcLowerBound.cost
                        || opportunityCost > bestArcLowerBound.opportunityCost) {
                    bestArcLowerBound.cost = cost;
                    bestArcLowerBound.opportunityCost = opportunityCost;
                    bestArcLowerBound.x = row;
                    bestArcLowerBound.y = col;
                }
            }
        }

        return {
            rowIndex: bestArcLowerBound.x,
            colIndex: bestArcLowerBound.y,
            cost: bestArcLowerBound.cost,
            opportunityCost: bestArcLowerBound.opportunityCost
        };
    }

    #getFirstTwoSmallestValues(matrix) {
        let height = matrix.length;
        let width = matrix[0].length;
        let rowsMinimum = [];
        let colsMinimum = [];

        let min1st, min2nd, row, col;
        for (row = 0; row != height; row += 1) {
            min1st = this.#MAX_VAL;
            min2nd = this.#MAX_VAL;

            for (col = 0; col != width; col += 1) {
                if (matrix[row][col] < min1st) {
                    min2nd = min1st;
                    min1st = matrix[row][col];
                } else if (matrix[row][col] < min2nd) {
                    min2nd = matrix[row][col];
                }
            }

            rowsMinimum.push({ first: min1st, second: min2nd });
        }

        for (col = 0; col != width; col += 1) {
            min1st = this.#MAX_VAL;
            min2nd = this.#MAX_VAL;

            for (row = 0; row != height; row += 1) {
                if (matrix[row][col] < min1st) {
                    min2nd = min1st;
                    min1st = matrix[row][col];
                } else if (matrix[row][col] < min2nd) {
                    min2nd = matrix[row][col];
                }
            }

            colsMinimum.push({ first: min1st, second: min2nd });
        }

        return { row: rowsMinimum, col: colsMinimum };
    }

    #getMatrixCopy(m) {
        let matrix = [];
        
        for (let row of m) {
            let newRow = [];

            for (let val of row) {
                newRow.push(val);
            }

            matrix.push(newRow);
        }

        return matrix;
    }

    #getInitNodeStack(m) {
        let height = m.length;
        let width = m[0].length;
        let rows = [];
        let cols = [];

        let i;
        for (i = 0; i != height; i += 1)
            rows.push(i);
        
        for (i = 0; i != width; i += 1)
            cols.push(i);
        
        return { rows: rows, cols: cols };
    }

    #setNoStandingStill(m) {
        let size = m.length;

        for (let i = 0; i != size; i += 1)
            m[i][i] = this.#MAX_VAL;
    }

    #lowerBoundProcess(m) {
        // console.log(m);
        let height = m.length;
        let width = m[0].length;
        let lowerBound = 0;

        let row, col;
        for (row = 0; row != height; row += 1) {
            let min = this.#MAX_VAL;

            for (col = 0; col != width; col += 1) {
                if (m[row][col] < min)
                    min = m[row][col];
            }

            if (!min)
                continue;

            for (col = 0; col != width; col += 1)
                m[row][col] -= min;

            lowerBound += min;
        }

        for (col = 0; col != width; col += 1) {
            let min = this.#MAX_VAL;

            for (row = 0; row != height; row += 1) {
                if (m[row][col] < min)
                    min = m[row][col];
            }

            if (!min)
                continue;
            
            for (row = 0; row != height; row += 1)
                m[row][col] -= min;
            
            lowerBound += min;
        }

        return lowerBound;

    }

    #initOptimalPath(len) {
        let i;

        for (i = 0; i != len; i += 1) {
            this.#pOptimalPath.push(this.#DUMMY);
            this.#pCurrentOptimalPath.push(this.#DUMMY);
        }
    }

};