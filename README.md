sl - DEPRECATED
=====
Use tonyrog/uart instead, a cross platform library (read, windows support)


### Dependencies

To build sl you will need a working installation of Erlang R15B (or
later).<br/>
Information on building and installing [Erlang/OTP](http://www.erlang.org)
can be found [here](https://github.com/erlang/otp/wiki/Installation)
([more info](https://github.com/erlang/otp/blob/master/INSTALL.md)).

sl is built using rebar that can be found [here](https://github.com/basho/rebar), with building instructions [here](https://github.com/basho/rebar/wiki/Building-rebar).

sl also requires the following application to be installed:
<ul>
<li>eapi - https://github.com/tonyrog/eapi</li>
</ul>


### Downloading

Clone the repository in a suitable location:

```sh
$ git clone git://github.com/tonyrog/sl.git
```
### Configurating
#### Concepts

...

#### Files

...

### Building

Rebar will compile all needed dependencies.<br/>
Compile:

```sh
$ cd sl
$ rebar compile
...
==> sl (compile)
```

