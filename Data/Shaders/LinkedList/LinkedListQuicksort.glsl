//const int STACK_SIZE = 18; // ceil(log2(MAX_NUM_FRAGS) / 2) * 2 * 2
int stackMemory[STACK_SIZE];
int stackCounter = 0;

void stackPush(int value) {
    //assert(stackCounter < STACK_SIZE);
    if (stackCounter < STACK_SIZE) {
        stackMemory[stackCounter] = value;
        stackCounter++;
    }
}

int stackPop() {
    //assert(stackCounter > 0);
    if (stackCounter > 0) {
        stackCounter--;
        return stackMemory[stackCounter];
    } else {
        return 0;
    }
}

bool stackEmpty() {
    return stackCounter == 0;
}

int partitionQuicksortLomuto(int low, int high) {
    float pivotElement = depthList[high];
    int i = low;
    for (int j = low; j <= high; j++) {
        if (depthList[j] < pivotElement) {
            swapFragments(i, j);
            i++;
        }
    }
    swapFragments(i, high);
    return i;
}

int partitionQuicksortHoare(int low, int high) {
    // Take first, middle, and last element. Then, use the median as the pivot element.
    float e0 = depthList[low];
    float e1 = depthList[(low + high) / 2];
    float e2 = depthList[high];
    float pivotElement = e0 < e1 ? (e2 < e0 ? e0 : min(e1, e2)) : (e2 < e1 ? e1 : min(e0, e2));

    int i = low - 1;
    int j = high + 1;
    while (true) {
        do {
            i = i + 1;
        } while (depthList[i] < pivotElement);
        do {
            j = j - 1;
        } while (depthList[j] > pivotElement);
        if (i >= j) {
            return j;
        }
        swapFragments(i, j);
    }
    return 0;
}

vec4 quicksort(uint fragsCount) {
    stackPush(0);
    stackPush(int(fragsCount) - 1);

    while (!stackEmpty()) {
        int high = stackPop();
        int low = stackPop();

        int pivot = partitionQuicksortLomuto(low, high);

        if (low < pivot - 1) {
            stackPush(low);
            stackPush(pivot - 1);
        }

        if (pivot + 1 < high) {
            stackPush(pivot + 1);
            stackPush(high);
        }
    }

    return blendFTB(fragsCount);
}

vec4 quicksortHybrid(uint fragsCount) {
    stackPush(0);
    stackPush(int(fragsCount) - 1);

    if (fragsCount > 16) {
        while (!stackEmpty()) {
            int high = stackPop();
            int low = stackPop();

            int pivot = partitionQuicksortHoare(low, high);

            if (low + 16 < pivot) {
                stackPush(low);
                stackPush(pivot - 1);
            }

            if (pivot + 16 < high) {
                stackPush(pivot + 1);
                stackPush(high);
            }
        }
    }
    insertionSort(fragsCount);

    return blendFTB(fragsCount);
}
