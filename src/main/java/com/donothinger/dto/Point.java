package com.donothinger.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.EqualsAndHashCode;

@AllArgsConstructor
@Data
@EqualsAndHashCode(exclude = { "x", "y", "z" })
public class Point {
    public Integer x;
    public Integer y;
    public Integer z;
}
