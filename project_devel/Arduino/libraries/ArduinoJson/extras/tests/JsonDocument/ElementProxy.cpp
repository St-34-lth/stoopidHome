// ArduinoJson - https://arduinojson.org
// Copyright © 2014-2025, Benoit BLANCHON
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

#include "Allocators.hpp"
#include "Literals.hpp"

using ElementProxy = ArduinoJson::detail::ElementProxy<JsonDocument&>;

TEST_CASE("ElementProxy::add()") {
  SpyingAllocator spy;
  JsonDocument doc(&spy);
  doc.add<JsonVariant>();
  const ElementProxy& ep = doc[0];

  SECTION("integer") {
    ep.add(42);

    REQUIRE(doc.as<std::string>() == "[[42]]");
    REQUIRE(spy.log() == AllocatorLog{
                             Allocate(sizeofPool()),
                         });
  }

  SECTION("string literal") {
    ep.add("world");

    REQUIRE(doc.as<std::string>() == "[[\"world\"]]");
    REQUIRE(spy.log() == AllocatorLog{
                             Allocate(sizeofPool()),
                         });
  }

  SECTION("const char*") {
    const char* s = "world";
    ep.add(s);

    REQUIRE(doc.as<std::string>() == "[[\"world\"]]");
    REQUIRE(spy.log() == AllocatorLog{
                             Allocate(sizeofPool()),
                             Allocate(sizeofString("world")),
                         });
  }

  SECTION("char[]") {
    char s[] = "world";
    ep.add(s);
    strcpy(s, "!!!!!");

    REQUIRE(doc.as<std::string>() == "[[\"world\"]]");
    REQUIRE(spy.log() == AllocatorLog{
                             Allocate(sizeofPool()),
                             Allocate(sizeofString("world")),
                         });
  }

#ifdef HAS_VARIABLE_LENGTH_ARRAY
  SECTION("VLA") {
    size_t i = 8;
    char vla[i];
    strcpy(vla, "world");

    ep.add(vla);

    REQUIRE(doc.as<std::string>() == "[[\"world\"]]");
    REQUIRE(spy.log() == AllocatorLog{
                             Allocate(sizeofPool()),
                             Allocate(sizeofString("world")),
                         });
  }
#endif
}

TEST_CASE("ElementProxy::clear()") {
  JsonDocument doc;
  doc.add<JsonVariant>();
  const ElementProxy& ep = doc[0];

  SECTION("size goes back to zero") {
    ep.add(42);
    ep.clear();

    REQUIRE(ep.size() == 0);
  }

  SECTION("isNull() return true") {
    ep.add("hello");
    ep.clear();

    REQUIRE(ep.isNull() == true);
  }
}

TEST_CASE("ElementProxy::operator==()") {
  JsonDocument doc;

  SECTION("1 vs 1") {
    doc.add(1);
    doc.add(1);

    REQUIRE(doc[0] <= doc[1]);
    REQUIRE(doc[0] == doc[1]);
    REQUIRE(doc[0] >= doc[1]);
    REQUIRE_FALSE(doc[0] != doc[1]);
    REQUIRE_FALSE(doc[0] < doc[1]);
    REQUIRE_FALSE(doc[0] > doc[1]);
  }

  SECTION("1 vs 2") {
    doc.add(1);
    doc.add(2);

    REQUIRE(doc[0] != doc[1]);
    REQUIRE(doc[0] < doc[1]);
    REQUIRE(doc[0] <= doc[1]);
    REQUIRE_FALSE(doc[0] == doc[1]);
    REQUIRE_FALSE(doc[0] > doc[1]);
    REQUIRE_FALSE(doc[0] >= doc[1]);
  }

  SECTION("'abc' vs 'bcd'") {
    doc.add("abc");
    doc.add("bcd");

    REQUIRE(doc[0] != doc[1]);
    REQUIRE(doc[0] < doc[1]);
    REQUIRE(doc[0] <= doc[1]);
    REQUIRE_FALSE(doc[0] == doc[1]);
    REQUIRE_FALSE(doc[0] > doc[1]);
    REQUIRE_FALSE(doc[0] >= doc[1]);
  }
}

TEST_CASE("ElementProxy::remove()") {
  JsonDocument doc;
  doc.add<JsonVariant>();
  const ElementProxy& ep = doc[0];

  SECTION("remove(int)") {
    ep.add(1);
    ep.add(2);
    ep.add(3);

    ep.remove(1);

    REQUIRE(ep.as<std::string>() == "[1,3]");
  }

  SECTION("remove(const char *)") {
    ep["a"] = 1;
    ep["b"] = 2;

    ep.remove("a");

    REQUIRE(ep.as<std::string>() == "{\"b\":2}");
  }

  SECTION("remove(std::string)") {
    ep["a"] = 1;
    ep["b"] = 2;

    ep.remove("b"_s);

    REQUIRE(ep.as<std::string>() == "{\"a\":1}");
  }

#ifdef HAS_VARIABLE_LENGTH_ARRAY
  SECTION("remove(vla)") {
    ep["a"] = 1;
    ep["b"] = 2;

    size_t i = 4;
    char vla[i];
    strcpy(vla, "b");
    ep.remove(vla);

    REQUIRE(ep.as<std::string>() == "{\"a\":1}");
  }
#endif
}

TEST_CASE("ElementProxy::set()") {
  JsonDocument doc;
  const ElementProxy& ep = doc[0];

  SECTION("set(int)") {
    ep.set(42);

    REQUIRE(doc.as<std::string>() == "[42]");
  }

  SECTION("set(const char*)") {
    ep.set("world");

    REQUIRE(doc.as<std::string>() == "[\"world\"]");
  }

  SECTION("set(char[])") {
    char s[] = "world";
    ep.set(s);
    strcpy(s, "!!!!!");

    REQUIRE(doc.as<std::string>() == "[\"world\"]");
  }

#ifdef HAS_VARIABLE_LENGTH_ARRAY
  SECTION("set(VLA)") {
    size_t i = 8;
    char vla[i];
    strcpy(vla, "world");

    ep.set(vla);

    REQUIRE(doc.as<std::string>() == "[\"world\"]");
  }
#endif
}

TEST_CASE("ElementProxy::size()") {
  JsonDocument doc;
  doc.add<JsonVariant>();
  const ElementProxy& ep = doc[0];

  SECTION("returns 0") {
    REQUIRE(ep.size() == 0);
  }

  SECTION("as an array, returns 2") {
    ep.add(1);
    ep.add(2);
    REQUIRE(ep.size() == 2);
  }

  SECTION("as an object, returns 2") {
    ep["a"] = 1;
    ep["b"] = 2;
    REQUIRE(ep.size() == 2);
  }
}

TEST_CASE("ElementProxy::operator[]") {
  JsonDocument doc;
  const ElementProxy& ep = doc[1];

  SECTION("set member") {
    ep["world"] = 42;

    REQUIRE(doc.as<std::string>() == "[null,{\"world\":42}]");
  }

  SECTION("set element") {
    ep[2] = 42;

    REQUIRE(doc.as<std::string>() == "[null,[null,null,42]]");
  }

#ifdef HAS_VARIABLE_LENGTH_ARRAY
  SECTION("set VLA") {
    size_t i = 8;
    char vla[i];
    strcpy(vla, "world");

    ep[0] = vla;

    REQUIRE(doc.as<std::string>() == "[null,[\"world\"]]");
  }
#endif
}

TEST_CASE("ElementProxy cast to JsonVariantConst") {
  JsonDocument doc;
  doc[0] = "world";

  const ElementProxy& ep = doc[0];

  JsonVariantConst var = ep;

  CHECK(var.as<std::string>() == "world");
}

TEST_CASE("ElementProxy cast to JsonVariant") {
  JsonDocument doc;
  doc[0] = "world";

  const ElementProxy& ep = doc[0];

  JsonVariant var = ep;

  CHECK(var.as<std::string>() == "world");

  var.set("toto");

  CHECK(doc.as<std::string>() == "[\"toto\"]");
}
