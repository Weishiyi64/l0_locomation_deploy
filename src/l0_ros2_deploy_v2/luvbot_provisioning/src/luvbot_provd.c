/* luvbot_provd.c —— BLE WiFi 配网守护进程（BlueZ + GDBus，纯 C）
 *
 * 编译： gcc -O2 -Wall luvbot_provd.c -o luvbot_provd $(pkg-config --cflags --libs gio-2.0)
 * 运行： sudo ./luvbot_provd --sn RUMI-TEST-007
 *
 * 协议： 手机 App 向 FFF1 特征写入 JSON：{"ssid":"WiFi名","pwd":"密码"}
 * 读取 FFF1 获取 UTF-8 JSON：{"status":"RUMI-TEST-007"}，status 字段承载 SN，不是配网状态
 * 读取只读 FFF2 获取 UTF-8 SN（无换行、无 NUL），蓝牙连接后即可读取，不依赖 WiFi
 *
 * [CHG] 生命周期：
 *   - 未连接 WiFi 时持续等待 BLE 操作，不因空闲或连接失败自动退出
 *   - WiFi 连接失败后保留广播，允许手机重新提交配网信息
 *   - 配网成功 → 立即通知 ROS，保持 BLE 广播 2s（仍可读 SN）后退出，退出码 0
 *   - 进入终态保持期后，手机的新写入被忽略（状态不再变化）
 * [CHG] 结果上报约定（只把成功交给 ROS2，BLE Read 仅提供 SN）：
 *   退出码：0=成功；启动错误或外部停止使用非零退出码
 *   成功时立即 stdout 输出 "[RESULT] ok"，拉起方据此发布 ROS2 话题通知同事
 */
#include <gio/gio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <glib-unix.h>

#ifndef L0_APP_VERSION
#define L0_APP_VERSION "unbuilt"
#endif

#define BLUEZ "org.bluez"
#define OM_IFACE "org.freedesktop.DBus.ObjectManager"
#define PROP_IFACE "org.freedesktop.DBus.Properties"
#define ADAPTER_PATH "/org/bluez/hci0"
#define APP_ROOT "/org/luvbot"
#define ADV_PATH APP_ROOT "/adv0"
#define SVC_PATH APP_ROOT "/service0"
#define CHR_PATH SVC_PATH "/char0"
#define SN_PATH SVC_PATH "/char1"
#define GATT_MGR "org.bluez.GattManager1"
#define ADV_MGR "org.bluez.LEAdvertisingManager1"
#define ADV_IFACE "org.bluez.LEAdvertisement1"
#define SVC_IFACE "org.bluez.GattService1"
#define CHR_IFACE "org.bluez.GattCharacteristic1"
#define SVC_UUID "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHR_UUID "0000fff1-0000-1000-8000-00805f9b34fb"
#define SN_UUID "0000fff2-0000-1000-8000-00805f9b34fb"
#define LOCAL_NAME "LUVRUMI"
static const gchar *broadcast_name = LOCAL_NAME;
static const gchar *device_sn = NULL;

/* ===================== [CHG] 生命周期参数 ===================== */
#define SUCCESS_HOLD_SEC  2     /* 成功后继续广播，供手机读取 SN */
#define EXIT_OK           0
/* ============================================================== */

static GMainLoop *loop;
static gboolean app_published = FALSE, adv_registered = FALSE, app_registered = FALSE;
static gchar g_status[128] = "idle";
static GSubprocess *wifi_process = NULL;

/* ===================== [CHG] 生命周期状态 ===================== */
static guint exit_timer_id     = 0;   /* 成功后退出定时器 */
static int   pending_exit_code = EXIT_OK;
/* ============================================================== */

static gboolean try_register(gpointer user_data);
static void register_next(GDBusConnection *conn);
static void schedule_retry(GDBusConnection *conn);

/* ===================== [CHG] 生命周期管理 ===================== */
static gboolean on_exit_timer(gpointer ud)
{
    exit_timer_id = 0;
    g_main_loop_quit(loop);
    return FALSE;
}

/* WiFi 成功后退出调度（只排一次） */
static void schedule_exit(unsigned int sec, int code)
{
    if (exit_timer_id) return;
    pending_exit_code = code;
    exit_timer_id = g_timeout_add(sec * 1000, on_exit_timer, NULL);
}
/* ============================================================== */

/* ---------------- WiFi 联网 ---------------- */
static void on_nmcli_done(GObject *src, GAsyncResult *res, gpointer user_data)
{
    GSubprocess *p = G_SUBPROCESS(src);
    GError *err = NULL;
    wifi_process = NULL;
    if (!g_subprocess_wait_finish(p, res, &err)) {
        snprintf(g_status, sizeof(g_status), "err");
        g_printerr("[PROV] nmcli wait failed: %s\n", err->message);
        g_clear_error(&err);
    } else if (g_subprocess_get_if_exited(p) && g_subprocess_get_exit_status(p) == 0) {
        snprintf(g_status, sizeof(g_status), "ok");
        g_print("[PROV] WiFi connect OK\n");
        g_print("[RESULT] ok\n");
        fflush(stdout);
        /* Keep BLE advertising briefly so the phone can still read the SN. */
        schedule_exit(SUCCESS_HOLD_SEC, EXIT_OK);
    } else {
        snprintf(g_status, sizeof(g_status), "err");
        g_printerr("[PROV] WiFi connect FAILED (exit=%d)\n", g_subprocess_get_if_exited(p) ? g_subprocess_get_exit_status(p) : -1);
        g_print("[PROV] 保持 BLE 广播，等待手机重新提交 WiFi 信息\n");
    }
    g_object_unref(p);
}

static void wifi_child_setup(gpointer unused)
{
    /* Give cancellation a process group containing the shell and every nmcli retry. */
    if (setpgid(0, 0) < 0) _exit(126);
}

static void start_wifi_connect(const gchar *ssid, const gchar *pwd)
{
    GError *err = NULL;
    gchar *q_ssid = g_shell_quote(ssid);
    gchar *q_pwd = g_shell_quote(pwd);

    gchar *line = g_strdup_printf(
        "nmcli dev wifi rescan 2>/dev/null; "
        "sleep 2; "
        "for i in 1 2 3; do "
        "  nmcli --wait 20 dev wifi connect %s password %s && exit 0; "
        "  sleep 2; "
        "done; exit 1",
        q_ssid, q_pwd);
    g_free(q_ssid); g_free(q_pwd);

    GSubprocessLauncher *launcher = g_subprocess_launcher_new(G_SUBPROCESS_FLAGS_NONE);
    g_subprocess_launcher_set_child_setup(launcher, wifi_child_setup, NULL, NULL);
    GSubprocess *p = g_subprocess_launcher_spawn(launcher, &err, "sh", "-c", line, NULL);
    g_object_unref(launcher);
    g_free(line);
    if (!p) {
        g_printerr("[PROV] spawn nmcli failed: %s\n", err->message);
        snprintf(g_status, sizeof(g_status), "spawn-fail");
        g_clear_error(&err);
        return;
    }
    wifi_process = p;
    g_subprocess_wait_async(p, NULL, on_nmcli_done, NULL);
}

/* shortcut: requires compact, unescaped string values; use a JSON library if the App protocol expands. */
static gchar *json_get_string(const gchar *buf, const gchar *key)
{
    gchar search[64];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    gchar *start = strstr(buf, search);
    if (!start) return NULL;
    start += strlen(search);
    gchar *end = strchr(start, '"');
    if (!end) return NULL;
    for (const gchar *cursor = start; cursor < end; ++cursor) {
        if (*cursor == '\\' || (guchar)*cursor < 0x20) return NULL;
    }
    return g_strndup(start, end - start);
}

static void handle_payload(const guchar *data, gsize len)
{
    if (len < 10 || len > 512 || memchr(data, '\0', len) != NULL) {
        snprintf(g_status, sizeof(g_status), "bad-format");
        return;
    }
    gchar *buf = g_strndup((const gchar *)data, len);

    if (buf[0] != '{' || buf[len-1] != '}') {
        g_printerr("[PROV] bad format, %zu bytes\n", len);
        snprintf(g_status, sizeof(g_status), "bad-format");
        g_free(buf);
        return;
    }

    gchar *ssid = json_get_string(buf, "ssid");
    gchar *pwd  = json_get_string(buf, "pwd");

    if (!ssid || strlen(ssid) == 0) {
        g_printerr("[PROV] missing ssid\n");
        snprintf(g_status, sizeof(g_status), "no-ssid");
        g_free(ssid); g_free(pwd); g_free(buf);
        return;
    }
    if (!pwd || strlen(pwd) == 0) {
        g_printerr("[PROV] missing pwd\n");
        snprintf(g_status, sizeof(g_status), "no-pwd");
        g_free(ssid); g_free(pwd); g_free(buf);
        return;
    }

    g_print("[PROV] RX ssid=\"%s\" pwd_len=%zu\n", ssid, strlen(pwd));
    snprintf(g_status, sizeof(g_status), "connecting");
    start_wifi_connect(ssid, pwd);

    g_free(ssid); g_free(pwd); g_free(buf);
}

/* ---------------- GVariant 构造 ---------------- */
static GVariant *empty_obj_array(void)
{
    return g_variant_new_array(G_VARIANT_TYPE_OBJECT_PATH, NULL, 0);
}

static GVariant *empty_options(void)
{
    GVariantBuilder b;
    g_variant_builder_init(&b, G_VARIANT_TYPE("a{sv}"));
    return g_variant_builder_end(&b);
}

static GVariant *build_managed_objects(void)
{
    GVariantBuilder objs, ifaces, props;

    g_variant_builder_init(&objs, G_VARIANT_TYPE("a{oa{sa{sv}}}"));

    g_variant_builder_init(&props, G_VARIANT_TYPE("a{sv}"));
    g_variant_builder_add(&props, "{sv}", "UUID", g_variant_new_string(SVC_UUID));
    g_variant_builder_add(&props, "{sv}", "Primary", g_variant_new_boolean(TRUE));
    g_variant_builder_add(&props, "{sv}", "Includes", empty_obj_array());
    g_variant_builder_init(&ifaces, G_VARIANT_TYPE("a{sa{sv}}"));
    g_variant_builder_add(&ifaces, "{s@a{sv}}", SVC_IFACE, g_variant_builder_end(&props));
    g_variant_builder_add(&objs, "{o@a{sa{sv}}}", SVC_PATH, g_variant_builder_end(&ifaces));

    const gchar *flags[] = { "read", "write", "write-without-response" };
    g_variant_builder_init(&props, G_VARIANT_TYPE("a{sv}"));
    g_variant_builder_add(&props, "{sv}", "UUID", g_variant_new_string(CHR_UUID));
    g_variant_builder_add(&props, "{sv}", "Service", g_variant_new_object_path(SVC_PATH));
    g_variant_builder_add(&props, "{sv}", "Flags", g_variant_new_strv(flags, G_N_ELEMENTS(flags)));
    g_variant_builder_init(&ifaces, G_VARIANT_TYPE("a{sa{sv}}"));
    g_variant_builder_add(&ifaces, "{s@a{sv}}", CHR_IFACE, g_variant_builder_end(&props));
    g_variant_builder_add(&objs, "{o@a{sa{sv}}}", CHR_PATH, g_variant_builder_end(&ifaces));

    const gchar *sn_flags[] = { "read" };
    g_variant_builder_init(&props, G_VARIANT_TYPE("a{sv}"));
    g_variant_builder_add(&props, "{sv}", "UUID", g_variant_new_string(SN_UUID));
    g_variant_builder_add(&props, "{sv}", "Service", g_variant_new_object_path(SVC_PATH));
    g_variant_builder_add(&props, "{sv}", "Flags", g_variant_new_strv(sn_flags, G_N_ELEMENTS(sn_flags)));
    g_variant_builder_init(&ifaces, G_VARIANT_TYPE("a{sa{sv}}"));
    g_variant_builder_add(&ifaces, "{s@a{sv}}", CHR_IFACE, g_variant_builder_end(&props));
    g_variant_builder_add(&objs, "{o@a{sa{sv}}}", SN_PATH, g_variant_builder_end(&ifaces));

    return g_variant_builder_end(&objs);
}

/* ---------------- introspection XML ---------------- */
static const gchar root_xml[] =
    "<node><interface name='" OM_IFACE "'>"
    "<method name='GetManagedObjects'>"
    "<arg type='a{oa{sa{sv}}}' direction='out'/></method>"
    "</interface></node>";

static const gchar svc_xml[] =
    "<node><interface name='" SVC_IFACE "'>"
    "<property name='UUID' type='s' access='read'/>"
    "<property name='Primary' type='b' access='read'/>"
    "<property name='Includes' type='ao' access='read'/>"
    "</interface></node>";

static const gchar chr_xml[] =
    "<node><interface name='" CHR_IFACE "'>"
    "<method name='ReadValue'>"
    "<arg type='a{sv}' direction='in'/><arg type='ay' direction='out'/></method>"
    "<method name='WriteValue'>"
    "<arg type='ay' direction='in'/><arg type='a{sv}' direction='in'/></method>"
    "<property name='UUID' type='s' access='read'/>"
    "<property name='Service' type='o' access='read'/>"
    "<property name='Flags' type='as' access='read'/>"
    "</interface></node>";

static const gchar adv_xml[] =
    "<node><interface name='" ADV_IFACE "'>"
    "<method name='Release'/>"
    "<property name='Type' type='s' access='read'/>"
    "<property name='ServiceUUIDs' type='as' access='read'/>"
    "<property name='LocalName' type='s' access='read'/>"
    "<property name='IncludeTxPower' type='b' access='read'/>"
    "<property name='Discoverable' type='b' access='read'/>"
    "</interface></node>";

static GDBusNodeInfo *root_info, *svc_info, *chr_info, *adv_info;

/* ---------------- 回调 ---------------- */
static void root_method(GDBusConnection *c, const gchar *sender, const gchar *path,
                        const gchar *iface, const gchar *method, GVariant *params,
                        GDBusMethodInvocation *inv, gpointer ud)
{
    if (g_strcmp0(method, "GetManagedObjects") == 0) {
        g_print("[OM] BlueZ 枚举服务对象\n");
        g_dbus_method_invocation_return_value(inv,
            g_variant_new("(@a{oa{sa{sv}}})", build_managed_objects()));
    } else {
        g_dbus_method_invocation_return_dbus_error(inv,
            "org.freedesktop.DBus.Error.UnknownMethod", method);
    }
}

static void return_read_text(GVariant *params, GDBusMethodInvocation *inv, const gchar *value)
{
    GVariant *options = g_variant_get_child_value(params, 0);
    guint16 offset = 0;
    g_variant_lookup(options, "offset", "q", &offset);
    g_variant_unref(options);
    const gsize size = strlen(value);
    if (offset > size) {
        g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.InvalidOffset", "Read offset exceeds value length");
        return;
    }
    GVariantBuilder bytes;
    g_variant_builder_init(&bytes, G_VARIANT_TYPE("ay"));
    for (gsize i = offset; i < size; ++i)
        g_variant_builder_add(&bytes, "y", (guchar)value[i]);
    g_dbus_method_invocation_return_value(inv, g_variant_new("(ay)", &bytes));
}

static void chr_method(GDBusConnection *c, const gchar *sender, const gchar *path,
                       const gchar *iface, const gchar *method, GVariant *params,
                       GDBusMethodInvocation *inv, gpointer ud)
{
    if (g_strcmp0(path, SN_PATH) == 0) {
        if (g_strcmp0(method, "ReadValue") != 0) {
            g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.NotPermitted", "SN is read-only");
            return;
        }
        return_read_text(params, inv, device_sn);
        return;
    }
    if (g_strcmp0(method, "WriteValue") == 0) {
        /* [CHG] 成功保持期（退出倒计时中）：忽略新写入，
         * 内部状态保持不变，手机仍能读取 SN */
        if (exit_timer_id) {
            g_print("[PROV] 终态保持期，忽略新写入\n");
            g_dbus_method_invocation_return_value(inv, NULL);
            return;
        }
        if (wifi_process) {
            g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.InProgress", "WiFi connection in progress");
            return;
        }
        GVariant *ay = g_variant_get_child_value(params, 0);
        gsize n;
        const guchar *d = g_variant_get_fixed_array(ay, &n, 1);
        handle_payload(d, n);
        g_variant_unref(ay);
        g_dbus_method_invocation_return_value(inv, NULL);
    } else if (g_strcmp0(method, "ReadValue") == 0) {
        /* SN validation excludes quotes and escapes, so it can be embedded directly. */
        gchar *json = g_strdup_printf("{\"status\":\"%s\"}", device_sn);
        return_read_text(params, inv, json);
        g_free(json);
    } else {
        g_dbus_method_invocation_return_dbus_error(inv,
            "org.freedesktop.DBus.Error.UnknownMethod", method);
    }
}

static void adv_method(GDBusConnection *c, const gchar *sender, const gchar *path,
                       const gchar *iface, const gchar *method, GVariant *params,
                       GDBusMethodInvocation *inv, gpointer ud)
{
    if (g_strcmp0(method, "Release") == 0) {
        g_print("[ADV] released by BlueZ, 1 秒后重新注册\n");
        adv_registered = FALSE;
        g_timeout_add_seconds(1, try_register, c);
    }
    g_dbus_method_invocation_return_value(inv, NULL);
}

static GVariant *svc_getprop(GDBusConnection *c, const gchar *sender, const gchar *path,
                             const gchar *iface, const gchar *name, GError **err, gpointer ud)
{
    if (g_strcmp0(name, "UUID") == 0)    return g_variant_new_string(SVC_UUID);
    if (g_strcmp0(name, "Primary") == 0) return g_variant_new_boolean(TRUE);
    if (g_strcmp0(name, "Includes") == 0) return empty_obj_array();
    return NULL;
}

static GVariant *chr_getprop(GDBusConnection *c, const gchar *sender, const gchar *path,
                             const gchar *iface, const gchar *name, GError **err, gpointer ud)
{
    const gboolean is_sn = g_strcmp0(path, SN_PATH) == 0;
    if (g_strcmp0(name, "UUID") == 0)
        return g_variant_new_string(is_sn ? SN_UUID : CHR_UUID);
    if (g_strcmp0(name, "Service") == 0)
        return g_variant_new_object_path(SVC_PATH);
    if (g_strcmp0(name, "Flags") == 0) {
        if (is_sn) {
            const gchar *f[] = { "read" };
            return g_variant_new_strv(f, G_N_ELEMENTS(f));
        }
        const gchar *f[] = { "read", "write", "write-without-response" };
        return g_variant_new_strv(f, G_N_ELEMENTS(f));
    }
    return NULL;
}

static GVariant *adv_getprop(GDBusConnection *c, const gchar *sender, const gchar *path,
                             const gchar *iface, const gchar *name, GError **err, gpointer ud)
{
    if (g_strcmp0(name, "Type") == 0)
        return g_variant_new_string("peripheral");
    if (g_strcmp0(name, "ServiceUUIDs") == 0) {
        const gchar *u[] = { SVC_UUID };
        return g_variant_new_strv(u, G_N_ELEMENTS(u));
    }
    if (g_strcmp0(name, "LocalName") == 0)
        return g_variant_new_string(broadcast_name);
    if (g_strcmp0(name, "IncludeTxPower") == 0)
        return g_variant_new_boolean(TRUE);
    if (g_strcmp0(name, "Discoverable") == 0)
        return g_variant_new_boolean(TRUE);
    return NULL;
}

static const GDBusInterfaceVTable root_vt = { root_method, NULL, NULL };
static const GDBusInterfaceVTable svc_vt  = { NULL, svc_getprop, NULL };
static const GDBusInterfaceVTable chr_vt  = { chr_method, chr_getprop, NULL };
static const GDBusInterfaceVTable adv_vt  = { adv_method, adv_getprop, NULL };

/* ---------------- 注册流程（异步，失败自动重试） ---------------- */
static void schedule_retry(GDBusConnection *conn)
{
    g_timeout_add_seconds(2, try_register, conn);
}

static void on_gatt_registered(GObject *source, GAsyncResult *res, gpointer ud)
{
    GDBusConnection *conn = G_DBUS_CONNECTION(source);
    GError *err = NULL;
    GVariant *r = g_dbus_connection_call_finish(conn, res, &err);
    if (!r) {
        g_print("[GATT] wait: %s\n", err->message);
        g_clear_error(&err);
        schedule_retry(conn);
        return;
    }
    g_variant_unref(r);
    app_registered = TRUE;
    g_print("[GATT] application registered\n");
    register_next(conn);
}

static void on_adv_registered(GObject *source, GAsyncResult *res, gpointer ud)
{
    GDBusConnection *conn = G_DBUS_CONNECTION(source);
    GError *err = NULL;
    GVariant *r = g_dbus_connection_call_finish(conn, res, &err);
    if (!r) {
        g_print("[ADV] wait: %s\n", err->message);
        g_clear_error(&err);
        schedule_retry(conn);
        return;
    }
    g_variant_unref(r);
    adv_registered = TRUE;
    g_print("[ADV] \"%s\" advertising ON\n", broadcast_name);
}

static void register_next(GDBusConnection *conn)
{
    if (!app_registered) {
        g_dbus_connection_call(conn, BLUEZ, ADAPTER_PATH, GATT_MGR,
            "RegisterApplication",
            g_variant_new("(o@a{sv})", APP_ROOT, empty_options()),
            NULL, G_DBUS_CALL_FLAGS_NONE, -1, NULL, on_gatt_registered, NULL);
    } else if (!adv_registered) {
        g_dbus_connection_call(conn, BLUEZ, ADAPTER_PATH, ADV_MGR,
            "RegisterAdvertisement",
            g_variant_new("(o@a{sv})", ADV_PATH, empty_options()),
            NULL, G_DBUS_CALL_FLAGS_NONE, -1, NULL, on_adv_registered, NULL);
    }
}

static gboolean try_register(gpointer user_data)
{
    GDBusConnection *conn = user_data;

    g_dbus_connection_call_sync(conn, BLUEZ, ADAPTER_PATH, PROP_IFACE, "Set",
        g_variant_new("(ssv)", "org.bluez.Adapter1", "Powered",
                      g_variant_new_boolean(TRUE)),
        NULL, G_DBUS_CALL_FLAGS_NONE, -1, NULL, NULL);

    g_dbus_connection_call_sync(conn, BLUEZ, ADAPTER_PATH, PROP_IFACE, "Set",
        g_variant_new("(ssv)", "org.bluez.Adapter1", "Alias",
                      g_variant_new_string(broadcast_name)),
        NULL, G_DBUS_CALL_FLAGS_NONE, -1, NULL, NULL);

    if (!app_published) {
        if (!g_dbus_connection_register_object(conn, APP_ROOT,
                root_info->interfaces[0], &root_vt, NULL, NULL, NULL))
            g_printerr("[DBG] APP_ROOT 注册失败\n");
        if (!g_dbus_connection_register_object(conn, SVC_PATH,
                svc_info->interfaces[0], &svc_vt, NULL, NULL, NULL))
            g_printerr("[DBG] SVC_PATH 注册失败\n");
        if (!g_dbus_connection_register_object(conn, CHR_PATH,
                chr_info->interfaces[0], &chr_vt, NULL, NULL, NULL))
            g_printerr("[DBG] CHR_PATH 注册失败\n");
        if (!g_dbus_connection_register_object(conn, SN_PATH,
                chr_info->interfaces[0], &chr_vt, NULL, NULL, NULL)) {
            g_printerr("[PROV] SN characteristic registration failed\n");
            pending_exit_code = 1;
            g_main_loop_quit(loop);
            return FALSE;
        }
        if (!g_dbus_connection_register_object(conn, ADV_PATH,
                adv_info->interfaces[0], &adv_vt, NULL, NULL, NULL))
            g_printerr("[DBG] ADV_PATH 注册失败\n");
        app_published = TRUE;
    }

    register_next(conn);
    return FALSE;
}

/* bluetoothd 掉线/重启时自动重新注册 */
static void on_bluez_owner_changed(GDBusConnection *c, const gchar *sender,
                                   const gchar *path, const gchar *iface,
                                   const gchar *signal, GVariant *params, gpointer ud)
{
    const gchar *name, *old_owner, *new_owner;
    g_variant_get(params, "(&s&s&s)", &name, &old_owner, &new_owner);
    if (g_strcmp0(name, BLUEZ) != 0) return;
    if (*new_owner == '\0' && (app_registered || adv_registered)) {
        g_print("[BUS] bluetoothd 退出，2 秒后重新注册\n");
        app_registered = FALSE;
        adv_registered = FALSE;
        g_timeout_add_seconds(2, try_register, c);
    }
}

static gboolean on_stop_signal(gpointer code)
{
    pending_exit_code = GPOINTER_TO_INT(code);
    g_main_loop_quit(loop);
    return G_SOURCE_REMOVE;
}

int main(int argc, char **argv)
{
    /* stdout is a pipe under the ROS bridge: flush each startup/advertising log line. */
    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("[VERSION] luvbot_provd version=%s\n", L0_APP_VERSION);
    fflush(stdout);
    GError *err = NULL;

    for (int i = 1; i < argc; ++i) {
        if (g_strcmp0(argv[i], "--name") == 0 && i + 1 < argc) {
            broadcast_name = argv[++i];
        } else if (g_strcmp0(argv[i], "--sn") == 0 && i + 1 < argc) {
            device_sn = argv[++i];
        } else {
            g_printerr("Usage: %s --sn DEVICE_SN [--name BROADCAST_NAME]\n", argv[0]);
            return 2;
        }
    }
    if (broadcast_name[0] == '\0' || !g_utf8_validate(broadcast_name, -1, NULL)) {
        g_printerr("Bluetooth broadcast name must be non-empty valid UTF-8\n");
        return 2;
    }

    if (!device_sn || !*device_sn || strlen(device_sn) > 64 ||
        strspn(device_sn, "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") != strlen(device_sn)) {
        g_printerr("--sn must contain 1-64 ASCII letters, digits, _ or -\n");
        return 2;
    }

    g_print("[PROV] luvbot provisioning daemon started\n");
    g_print("[PROV] SN=%s; read characteristic %s\n", device_sn, SN_UUID);

    root_info = g_dbus_node_info_new_for_xml(root_xml, NULL);
    svc_info  = g_dbus_node_info_new_for_xml(svc_xml, NULL);
    chr_info  = g_dbus_node_info_new_for_xml(chr_xml, NULL);
    adv_info  = g_dbus_node_info_new_for_xml(adv_xml, NULL);
    if (!root_info || !svc_info || !chr_info || !adv_info) {
        g_printerr("introspection XML 解析失败\n");
        return 1;
    }

    GDBusConnection *conn = g_bus_get_sync(G_BUS_TYPE_SYSTEM, NULL, &err);
    if (!conn) {
        g_printerr("bus: %s\n", err->message);
        return 1;
    }

    g_dbus_connection_signal_subscribe(conn, "org.freedesktop.DBus",
        "org.freedesktop.DBus", "NameOwnerChanged",
        "/org/freedesktop/DBus", NULL, G_DBUS_SIGNAL_FLAGS_NONE,
        on_bluez_owner_changed, NULL, NULL);

    g_timeout_add_seconds(2, try_register, conn);

    loop = g_main_loop_new(NULL, FALSE);
    g_unix_signal_add(SIGTERM, on_stop_signal, GINT_TO_POINTER(143));
    g_unix_signal_add(SIGINT, on_stop_signal, GINT_TO_POINTER(130));
    g_main_loop_run(loop);
    if (wifi_process) {
        const gchar *identifier = g_subprocess_get_identifier(wifi_process);
        if (identifier) kill(-(pid_t)g_ascii_strtoll(identifier, NULL, 10), SIGTERM);
        g_subprocess_force_exit(wifi_process);
    }

    /* 仅 WiFi 成功正常退出；错误或外部停止不发布 ROS2 成功消息。 */
    g_print("[PROV] exit(%d)\n", pending_exit_code);
    return pending_exit_code;
}
