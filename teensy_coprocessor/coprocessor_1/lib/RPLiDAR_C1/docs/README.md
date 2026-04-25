
```mermaid
---
config:
    title: UART Decimation Filter
    class: invert
title: UART Decimation Filter
---
stateDiagram-v2
direction TB

    HOLD
    FIND_PATTERN
    state if_choice <<choice>>
    ADD_OFFSET
    SKIP
    RECORD


    [*] --> HOLD

    HOLD --> FIND_PATTERN: current_state == READY
    % note left of HOLD
    %   lim = WAIT_INDEX
    % end note

    FIND_PATTERN --> if_choice : lim == FIND_INDEX
    % note left of FIND_PATTERN
    %   lim = FIND_INDEX
    % end note

    if_choice --> FIND_PATTERN : pattern not found
    if_choice --> ADD_OFFSET: 0 < offset < MSG_LENGTH
    if_choice --> RECORD: offset == 0

    ADD_OFFSET --> RECORD : lim == offset
    % note left of ADD_OFFSET
    %   lim = offset
    % end note

    SKIP --> HOLD: counter > BUFFER_SIZE

    SKIP --> RECORD : lim == SKIP_INDEX
    % note left of SKIP
    %   lim = SKIP_INDEX
    % end note

    RECORD --> SKIP : lim == MSG_LENGTH
    % note right of RECORD
    %   lim = MSG_LENGTH
    % end note
```

Under the realm of real-time embedded systems, it would not be acceptable to simply capture all data and record every Mth value onto a new array when memory is at a premium. It is good design to process this data as soon as the train of pulses arrive.

The primary motivation, then, is to assign which range of byte indices to record and ignore. When visualizing this in a mermaid diagram, it is important to know what capture state we are at, and under what limit conditions facilitate transition.

In this case, we require a `counter` that counts bytes and resets when it reaches a `limit` assigned to each state. Each state has a different function.

`HOLD` is idle until an external signal arrives. `FIND_PATTERN` records 20 bytes and finds the starting index of the 5-byte message and remains at `FIND_PATTERN` until a pattern is found. If found, the counter counts to the given `offset` value in order for subsequent counters to arrive exactly at the first byte. It then cycles through `RECORD`, `SKIP`, and back to `RECORD`, reading the first 5 bytes and ignoring `M-1` messages, or ignoring `(M-1)*5` bytes. These messages are concatenated immediately to their angle-distance pairs in a container buffer.

The process restarts back to `HOLD` until this buffer is full.
