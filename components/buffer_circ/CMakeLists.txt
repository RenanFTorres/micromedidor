#if(CONFIG_BUFFER_CIRC)
#    set(src "buffer_circ.c")
#    set(inc "include")
#endif()

#idf_component_register(SRCS ${src}
#                    INCLUDE_DIRS ${inc}
#                    REQUIRES esp_http_server esp-tls
#                    PRIV_REQUIRES lwip)
