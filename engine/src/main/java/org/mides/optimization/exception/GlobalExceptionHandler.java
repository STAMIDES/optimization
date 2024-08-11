package org.mides.optimization.exception;

import org.mides.optimization.exception.osrm.QueryMatrixException;
import org.mides.optimization.exception.osrm.QueryRouteException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;

@RestControllerAdvice
public class GlobalExceptionHandler {

    @ExceptionHandler(QueryMatrixException.class)
    public ResponseEntity<ErrorResponse> handleQueryMatrixException(QueryMatrixException ex) {
        var error = new ErrorResponse(HttpStatus.INTERNAL_SERVER_ERROR.value(), ex.getMessage());
        return ResponseEntity.status(HttpStatus.OK).body(error);
    }

    @ExceptionHandler(QueryRouteException.class)
    public ResponseEntity<ErrorResponse> handleQueryRouteException(QueryRouteException ex) {
        var error = new ErrorResponse(HttpStatus.INTERNAL_SERVER_ERROR.value(), ex.getMessage());
        return ResponseEntity.status(HttpStatus.OK).body(error);
    }
}
