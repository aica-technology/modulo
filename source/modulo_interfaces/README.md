# Modulo Interfaces

This package defines custom standard interfaces for modulo classes.

## Messages

Modulo classes broadcast predicates to a global channel in a predicate message.

### Predicate

The predicate message contains the predicate name and the current value (true or false) of the predicate.

### PredicateCollection

The predicate collection message contains a vector of predicate messages as well as the name of the node that emits the
predicates.

## Services

Modulo classes provide a simplified method to add services which trigger a pre-defined callback function.
Services are either empty (with no request payload) or carry a string request. They return a common
modulo_components::ComponentServiceResponse structure with a success flag and status message.

### EmptyTrigger

The EmptyTrigger service request takes no parameters.
The response contains a boolean `success` flag and a string `message`.

### StringTrigger

The EmptyTrigger service request takes a string `payload`.
The response contains a boolean `success` flag and a string `message`.
It is the responsibility of the component to define the expected payload format and to document it appropriately.